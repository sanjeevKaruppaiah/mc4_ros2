# MC 4.0 with an ESP32 + RPLIDAR A1: wiring and first run

The reference build for this repo. Work through it top to bottom. Each step
has a check, so a fault shows up before it turns into a "SLAM doesn't work"
problem.

## Build sheet

| Parameter | Value | Set in |
|-----------|-------|--------|
| Chassis | 4WD skid steer | `four_wheel:=true` (default) |
| Wheel diameter / radius | 65 mm / 0.0325 m | `wheel_radius:=0.0325` (default) |
| Track width | 250 mm | `wheel_separation:=0.250` (default; tune in §5) |
| Wheelbase | 300 mm | `wheelbase:=0.300` (default) |
| Loaded motor RPM | **to measure** (§3) | firmware `LOADED_RPM` |
| Motor dead-band PWM | **to measure** (§3) | firmware `MIN_PWM_LEFT/RIGHT` |
| Encoders | none | firmware `HAS_ENCODERS 0`, `use_encoders:=false` (defaults) |
| Lidar height | 180 mm (example: **measure yours**) | `lidar_z:=0.180` |

## How it works without encoders

```
 cmd_vel ─► mc4_base_node ─► ESP32 ─► PWM (open loop, from LOADED_RPM + MIN_PWM)
                 │
                 └─► /wheel_odom  (dead reckoning, drifts, no TF, info only)

 RPLIDAR A1 ─► /scan ─► rf2o_laser_odometry ─► /odom + TF odom→base_footprint
                   └──► slam_toolbox ─► /map + TF map→odom ─► Nav2
```

The motors don't report how far they turned, so **the lidar measures motion**:
rf2o compares each scan with the previous one. Nav2 closes the loop, so a wheel
running a little slow is corrected automatically. Two limits apply:

* rf2o needs **walls or objects in view**. Long, featureless corridors or a
  wide-open hall confuse it. A walled competition arena is ideal.
* **Turn slowly.** The A1 takes about 5.5 scans per second, and fast spins smear
  the scans. `nav2.yaml` already caps rotation at 0.8 rad/s.

Adding wheel encoders later is the biggest upgrade. Set `HAS_ENCODERS 1` and
run with `use_encoders:=true odom_source:=wheel`.

## 1. Wiring

```
 Battery (7-12 V) ──► Motor driver ──► left pair  (2 motors in parallel, channel A)
                         ▲  PWM/DIR ──► right pair (2 motors in parallel, channel B)
                      ESP32 DevKit
                         │ USB (serial 115200)
 Raspberry Pi / laptop ◄─┤
   (ROS 2 Jazzy)         │ USB (CP2102 adapter, 115200)
                      RPLIDAR A1
```

| ESP32 pin | TB6612 | L298N | Function |
|-----------|--------|-------|----------|
| 25 | PWMA | ENA (remove jumper) | Left side speed (PWM) |
| 26 | AIN1 | IN1 | Left direction |
| 27 | AIN2 | IN2 | Left direction |
| 14 | PWMB | ENB (remove jumper) | Right side speed (PWM) |
| 16 | BIN1 | IN3 | Right direction |
| 17 | BIN2 | IN4 | Right direction |
| 3V3 | STBY + VCC | n/a | TB6612 logic, enable |
| GND | GND | GND | **Common ground with the battery** |

4WD note: four TT-style motors in parallel can draw 2-3 A at stall. A TB6612 is
rated at only 1.2 A per channel, so use an **L298N** or a higher-current driver
for a 4-motor chassis.

Pins 18/19 and 32/33 are reserved for encoders if you add them later. Pins
avoided on purpose: 0, 2, 5, 12, 15 (boot strapping), 6-11 (flash), 1 and 3
(USB serial), 34-39 (input-only).

**Lidar.** Plug the A1 into its USB adapter and the adapter into the Pi. Mount
it at the **centre of the robot, above every other part** (nothing may block
the beam), and measure the floor-to-scan-plane height for `lidar_z`. If it
spins unevenly or keeps disconnecting, use a powered USB hub.

## 2. Flash the ESP32

1. In Arduino IDE, add the ESP32 boards package (Espressif, 2.x or 3.x) and select **ESP32 Dev Module**.
2. Open `firmware/mc4_base_firmware/mc4_base_firmware.ino`. Check that `HAS_ENCODERS` is `0`, then upload.
3. Open Serial Monitor at **115200** with the "Newline" line ending. You should see
   `# MC4.0 base firmware ready (no encoders, open loop)`.

**Check: direction.** Lift the robot so the wheels are off the ground. Send
`P 150 150`. All four wheels should turn **forward** for half a second. If one
side runs backwards, flip its `INVERT_*_MOTOR` and upload again. If a single
wheel of a pair runs backwards, swap that motor's two wires.

Close Serial Monitor before the next step.

## 3. Measure the motor constants

On the Pi (or laptop) with the ESP32 plugged in:

```bash
# a) Dead-band: the lowest PWM that gets the robot rolling ON THE FLOOR
python3 scripts/calibrate_motors.py deadband --port /dev/ttyUSB0

# b) Loaded RPM: full power for 3 s, then enter the distance you measured
python3 scripts/calibrate_motors.py rpm --port /dev/ttyUSB0 --seconds 3
```

Example: the robot covers 1.5 m in 3 s → 1.5 / (π × 0.065) / 3 × 60 ≈ **147 RPM**.

Put the results into the firmware and upload again:

```cpp
const float LOADED_RPM    = 147.0;
const int   MIN_PWM_LEFT  = 75;
const int   MIN_PWM_RIGHT = 80;
```

If the robot curves during the RPM run, set `TRIM_LEFT`/`TRIM_RIGHT` below
1.0 on the faster side, and repeat until it runs roughly straight.

Then make sure `max_wheel_speed` in `src/mc4_bringup/config/base.yaml` is **at or below**
`LOADED_RPM × 2π / 60` (147 RPM → 15.4 rad/s).

## 4. USB names on the Pi

```bash
bash scripts/list_serial.sh
```

Edit `scripts/99-mc4.rules`: keep **Case A** if the ESP32 shows `1a86:*`, or
**Case B** if it shows `10c4:ea60` (the same chip as the lidar). Then:

```bash
sudo cp scripts/99-mc4.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
ls -l /dev/mc4_base /dev/mc4_lidar
```

## 5. First ROS run

```bash
ros2 launch mc4_bringup robot.launch.py \
  base_port:=/dev/mc4_base lidar_port:=/dev/mc4_lidar lidar_z:=<your height>
```

Check that the expected topics are running:

```bash
ros2 topic hz /scan    # about 5.5 Hz from the RPLIDAR A1
ros2 topic hz /odom    # published by rf2o, not by the base
```

### Check lidar orientation

Open RViz (`ros2 launch mc4_bringup rviz.launch.py`), set **Fixed Frame** to
`base_footprint`, and put a box about 0.5 m **in front of** the robot.

* The red points appear in front (along the red X axis): correct.
* They appear **behind** the robot: add `lidar_yaw:=3.14159`.
* They appear on the wrong side (mirrored): set `'inverted': True` for `rplidar_a1` in `launch/lidar.launch.py`.

### Check lidar odometry

Set **Fixed Frame** to `odom` and drive with the keyboard
(`ros2 run teleop_twist_keyboard teleop_twist_keyboard`; press `z` a few times to slow down).

Drive 1 m forward next to a wall: the RobotModel in RViz should move about 1 m,
and the wall should stay still. If the robot jumps or the wall slides, slow
down and make sure walls are in view.

### Tune turning (skid steer)

A skid-steer robot turns slower than the geometry says, because its wheels
scrub sideways. Spin it on the spot for 10 s:

```bash
timeout 10 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
```

0.5 rad/s for 10 s should be 5 rad, about 0.8 of a full turn. If the robot
turns less, raise `wheel_separation` by the same ratio. For example, if it
only managed 0.5 turn, use 0.25 × 0.8 / 0.5 = **0.40**. Typical values are
1.2-1.8 × the real 250 mm track. Nav2 corrects small errors on its own; this
step just makes turns smoother.

## 6. Map, then navigate

```bash
ros2 launch mc4_bringup robot.launch.py base_port:=/dev/mc4_base lidar_port:=/dev/mc4_lidar
ros2 launch mc4_bringup navigation.launch.py slam:=true
ros2 launch mc4_bringup rviz.launch.py          # laptop
```

Click **2D Goal Pose** in RViz. Save the map with
`ros2 run nav2_map_server map_saver_cli -f ~/maps/arena`.

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| `Cannot open /dev/mc4_base` | udev rule not matched (`ls /dev/mc4_*`), or Serial Monitor / calibrate script still open |
| Robot hums but does not move at slow speeds | `MIN_PWM_*` too low: redo §3a |
| Robot overshoots goals / jerky | `LOADED_RPM` too low (robot faster than it thinks): redo §3b |
| Map smears or doubles when turning | Turning too fast, or lidar odometry lost walls. Slow down, stay near walls |
| Robot does not reach the heading at goals | Skid steer needs more torque to turn: raise `min_rotational_vel` or `MIN_PWM` |
| No `/odom` at all | rf2o not built: `vcs import src < src/mc4_ros2/deps.repos` and rebuild |
| RPLIDAR `SL_RESULT_OPERATION_TIMEOUT` | Weak USB power, or the wrong port given to the lidar |
| ESP32 won't boot when wired | A strapping pin is being pulled; use only the pins in the table |
