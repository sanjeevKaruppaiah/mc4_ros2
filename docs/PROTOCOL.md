# MC 4.0 ⇄ ROS 2 serial protocol

Plain ASCII lines, `\n` terminated, **115200 baud 8N1**. Human-readable on
purpose, so students can test it from the Arduino Serial Monitor before ROS
is involved.

## Host → MCU

| Line | Meaning |
|------|---------|
| `V <left> <right>` | Target wheel angular speeds in **rad/s** (float). Positive = robot moves forward. Sent at 30 Hz by `mc4_base_node`. |
| `P <left> <right>` | Raw PWM, -255..255 (integers). For bench tests and `scripts/calibrate_motors.py`. |
| `R` | Reset both encoder counters to 0. |

The firmware stops the motors if no `V`/`P` line arrives for **500 ms** (watchdog).

Without encoders (`HAS_ENCODERS 0`), `V` is turned into PWM open-loop:
`pwm = MIN_PWM + |speed| / MAX_WHEEL_RAD_S × (255 − MIN_PWM)`, where
`MAX_WHEEL_RAD_S = LOADED_RPM × 2π / 60`.

## MCU → Host

| Line | Meaning |
|------|---------|
| `E <left_ticks> <right_ticks>` | **Only with `HAS_ENCODERS 1`.** Cumulative signed encoder counts, sent every 20 ms. Forward motion = counts increase. |
| `# <text>` | Free-text log, shown in the ROS console. |

Cumulative counts (not deltas) mean a dropped line costs nothing: the next
line still carries the full distance travelled.

## Quick bench test (no ROS)

Open Serial Monitor at 115200 with "Newline" line ending:

```
P 150 150    -> both sides forward at PWM 150 (stops after 0.5 s)
V 3 -3       -> spin in place at 3 rad/s per wheel
R            -> zero the encoders (encoder build only)
```

If a side turns backwards, flip `INVERT_*_MOTOR` in the sketch. With
encoders, driving forward must make **both** `E` numbers grow; if one goes
down, flip `INVERT_*_ENC`.

## Calibrating `ticks_per_rev` (encoder build only)

1. Send `R`, then turn one wheel by hand exactly 10 revolutions.
2. `ticks_per_rev = E-value / 10`.
3. Put the same number in the firmware (`TICKS_PER_REV`) and in
   `mc4_bringup/config/base.yaml`.

## Replacing the protocol with micro-ROS

If your MC 4.0 controller is an ESP32 you can swap this bridge for
[micro-ROS](https://micro.ros.org/) later. Keep the same topics (`/cmd_vel` in,
`/odom` + TF out) and the rest of the stack (SLAM, Nav2, RViz) does not change.
