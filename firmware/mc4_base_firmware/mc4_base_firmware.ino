/*
 * MC 4.0 base firmware: ROS 2 bridge for a differential / skid-steer robot.
 *
 * Primary target: ESP32 DevKit (Arduino IDE board "ESP32 Dev Module", core 2.x / 3.x).
 * Also builds for Arduino Uno/Nano as a fallback.
 * Speaks the line protocol in docs/PROTOCOL.md at 115200 baud:
 *
 *   host -> MCU   V <left_rad_s> <right_rad_s>\n   target wheel speeds
 *                 P <left_pwm> <right_pwm>\n       raw PWM -255..255 (bench tests / calibration)
 *                 R\n                             reset encoder counts
 *   MCU  -> host  E <left_ticks> <right_ticks>\n  cumulative encoder counts (only if HAS_ENCODERS)
 *                 # <text>\n                      info/log line
 *
 * Two build modes:
 *   HAS_ENCODERS 0  open-loop: PWM is computed from the target speed using the
 *                   measured loaded RPM and the start-up PWM. ROS gets odometry
 *                   from the lidar (rf2o). <- default for the current MC 4.0 build
 *   HAS_ENCODERS 1  closed-loop PI speed control + E lines for wheel odometry.
 *
 * 4WD skid steer: wire the two left motors in parallel on driver channel A and
 * the two right motors on channel B.
 */

// ===================== Build mode ==========================================
#define HAS_ENCODERS 0

// ===================== Pin map (ESP32 DevKit / WROOM-32) ===================
// See docs/ESP32_RPLIDAR_A1.md for the wiring table.
// Avoided on purpose: 0, 2, 5, 12, 15 (boot strapping), 6-11 (flash),
// 1/3 (USB serial to ROS), 34-39 (input-only, NO internal pull-ups).
#if defined(ESP32)
  const int L_PWM = 25, L_IN1 = 26, L_IN2 = 27;   // left side motors
  const int R_PWM = 14, R_IN1 = 16, R_IN2 = 17;   // right side motors
  const int L_ENC_A = 18, L_ENC_B = 19;           // left encoder  (if fitted)
  const int R_ENC_A = 32, R_ENC_B = 33;           // right encoder (if fitted)
  #define ISR_ATTR IRAM_ATTR
#else  // Arduino Uno / Nano: only pins 2 and 3 have hardware interrupts
  const int L_PWM = 5,  L_IN1 = 7,  L_IN2 = 8;
  const int R_PWM = 6,  R_IN1 = 9,  R_IN2 = 10;
  const int L_ENC_A = 2, L_ENC_B = 4;
  const int R_ENC_A = 3, R_ENC_B = 11;
  #define ISR_ATTR
#endif

// ===================== Robot constants (measure! see docs) =================
// Loaded wheel RPM at PWM 255, measured ON THE FLOOR (docs/ESP32_RPLIDAR_A1.md section 2).
const float LOADED_RPM      = 150.0;                     // <-- replace with your measurement
const float MAX_WHEEL_RAD_S = LOADED_RPM * 2.0 * PI / 60.0;
// Lowest PWM that makes the robot start moving on the floor (motor dead-band).
const int   MIN_PWM_LEFT    = 70;                        // <-- measure with "P" command
const int   MIN_PWM_RIGHT   = 70;
// Trim if the robot curves when driving "straight" (1.0 = no change, e.g. 0.95 slows that side).
const float TRIM_LEFT       = 1.0;
const float TRIM_RIGHT      = 1.0;

const bool  INVERT_LEFT_MOTOR  = false; // flip if a side spins the wrong way
const bool  INVERT_RIGHT_MOTOR = true;

#if HAS_ENCODERS
const float TICKS_PER_REV    = 1320.0;  // encoder counts per WHEEL revolution (x2 decoding)
const bool  INVERT_LEFT_ENC  = false;
const bool  INVERT_RIGHT_ENC = true;
const float KP = 12.0, KI = 60.0;       // speed PI controller
#endif

const int   PWM_MAX = 255;
const unsigned long CONTROL_PERIOD_MS = 20;   // 50 Hz
const unsigned long CMD_TIMEOUT_MS    = 500;  // stop if host goes silent

// ===================== State ==============================================
float target[2] = {0, 0};      // rad/s from V command
int   rawPwm[2] = {0, 0};      // from P command
bool  rawMode   = false;
unsigned long lastControl = 0, lastCmd = 0;
char  lineBuf[48];
uint8_t lineLen = 0;

#if HAS_ENCODERS
volatile long encTicks[2] = {0, 0};
float integ[2]  = {0, 0};
long  prevTicks[2] = {0, 0};

// x2 decoding: count on every edge of channel A, direction from channel B.
void ISR_ATTR leftEncISR() {
  bool fwd = digitalRead(L_ENC_A) != digitalRead(L_ENC_B);
  encTicks[0] += (fwd ^ INVERT_LEFT_ENC) ? 1 : -1;
}
void ISR_ATTR rightEncISR() {
  bool fwd = digitalRead(R_ENC_A) != digitalRead(R_ENC_B);
  encTicks[1] += (fwd ^ INVERT_RIGHT_ENC) ? 1 : -1;
}
void readTicks(long out[2]) {
  noInterrupts();
  out[0] = encTicks[0];
  out[1] = encTicks[1];
  interrupts();
}
#endif

void setMotor(int pwmPin, int in1, int in2, int pwm, bool invert) {
  if (invert) pwm = -pwm;
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  digitalWrite(in1, pwm > 0 ? HIGH : LOW);
  digitalWrite(in2, pwm < 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(pwm));
}

// Open-loop speed -> PWM: linear between the dead-band and full speed.
int openLoopPwm(float radS, int minPwm, float trim) {
  if (radS == 0) return 0;
  float frac = constrain(fabs(radS) / MAX_WHEEL_RAD_S, 0.0, 1.0);
  int pwm = (int)((minPwm + frac * (PWM_MAX - minPwm)) * trim);
  return radS > 0 ? pwm : -pwm;
}

void handleLine(char *line) {
  char *p = line + 1;
  if (line[0] == 'V') {
    float l = strtod(p, &p);
    float r = strtod(p, &p);
    target[0] = constrain(l, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);
    target[1] = constrain(r, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);
    rawMode = false;
    lastCmd = millis();
  } else if (line[0] == 'P') {
    rawPwm[0] = constrain((int)strtol(p, &p, 10), -PWM_MAX, PWM_MAX);
    rawPwm[1] = constrain((int)strtol(p, &p, 10), -PWM_MAX, PWM_MAX);
    rawMode = true;
    lastCmd = millis();
  } else if (line[0] == 'R') {
#if HAS_ENCODERS
    noInterrupts();
    encTicks[0] = encTicks[1] = 0;
    interrupts();
    prevTicks[0] = prevTicks[1] = 0;
#endif
    Serial.println(F("# reset"));
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;  // overflow: drop the line
    }
  }
}

void setup() {
  Serial.begin(115200);
  int outs[] = {L_PWM, L_IN1, L_IN2, R_PWM, R_IN1, R_IN2};
  for (int pin : outs) pinMode(pin, OUTPUT);
#if HAS_ENCODERS
  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncISR, CHANGE);
  Serial.println(F("# MC4.0 base firmware ready (encoders, closed loop)"));
#else
  Serial.println(F("# MC4.0 base firmware ready (no encoders, open loop)"));
#endif
}

void loop() {
  readSerial();

  unsigned long now = millis();
  if (now - lastControl < CONTROL_PERIOD_MS) return;
  float dt = (now - lastControl) / 1000.0;
  lastControl = now;

  if (now - lastCmd > CMD_TIMEOUT_MS) {   // watchdog
    target[0] = target[1] = 0;
    rawPwm[0] = rawPwm[1] = 0;
  }

  int pwm[2];
  if (rawMode) {
    pwm[0] = rawPwm[0];
    pwm[1] = rawPwm[1];
  } else {
    pwm[0] = openLoopPwm(target[0], MIN_PWM_LEFT, TRIM_LEFT);
    pwm[1] = openLoopPwm(target[1], MIN_PWM_RIGHT, TRIM_RIGHT);
  }

#if HAS_ENCODERS
  long ticks[2];
  readTicks(ticks);
  for (int i = 0; i < 2; i++) {
    float measured = (ticks[i] - prevTicks[i]) * 2.0 * PI / TICKS_PER_REV / dt;
    prevTicks[i] = ticks[i];
    if (rawMode || target[i] == 0) {   // no correction in raw mode / when stopped
      integ[i] = 0;
      continue;
    }
    float err = target[i] - measured;
    integ[i] = constrain(integ[i] + err * dt, -4.0, 4.0);
    pwm[i] += (int)(KP * err + KI * integ[i]);
  }
#else
  (void)dt;
#endif

  setMotor(L_PWM, L_IN1, L_IN2, pwm[0], INVERT_LEFT_MOTOR);
  setMotor(R_PWM, R_IN1, R_IN2, pwm[1], INVERT_RIGHT_MOTOR);

#if HAS_ENCODERS
  Serial.print('E'); Serial.print(' ');
  Serial.print(ticks[0]); Serial.print(' ');
  Serial.println(ticks[1]);
#endif
}
