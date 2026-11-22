# GMRT Robotic Arm — Day 4 firmware

## Hardware configuration
- MCU: ESP32-WROOM-32 / ESP32 DevKit V1 compatible board
- Servos: 5 x SG90
- GPIO: J1 Base=13, J2 Shoulder=14, J3 Elbow=27, J4 Wrist=26, J5 Gripper=25
- Geometry (reported by mechanical team, cm): BASE_HEIGHT=8.0, L1=13.5, L2=7.0, L3=7.5

## Kinematics model and important assumption
The mechanical team reports J4 rotates the gripper. This firmware models J4 as wrist **roll about the distal arm axis**, so J4 changes tool roll but does not change XYZ. L2 and L3 are treated as collinear in the pitch plane; the position solver therefore uses L1 and (L2+L3) as its two planar links. This is a practical first model, not a verified CAD transform. If the physical wrist rotates about another axis or L3 is not collinear with the forearm, revise FK/IK after inspecting the actual joint axis.

The reported dimensions are accepted as measured values, but confirm that BASE_HEIGHT is base-axis center to shoulder-axis center, L1/L2 are joint-axis center distances, and L3 is wrist-axis center to gripper contact tip.

## Calibration
`CAL` in `src/main.cpp` is deliberately provisional: offset=90 degrees, direction=+1, command range=0..180. These are placeholders, not measured safe mechanical limits. Calibrate each servo with the horn/linkage disconnected, then set offset, direction and conservative per-joint min/max command values before attaching the mechanism. SG90s provide no position feedback.

## First power-up safety
1. Do not attach the servo horns/linkages for the first powered attachment test.
2. Verify polarity and measure the regulator output before connecting servos.
3. Do not power five SG90s from the ESP32 5V pin. Use a separately sized regulated servo supply and common ground with ESP32.
4. The BOM lists a 3S LiPo and LM2596 but does not establish that the particular buck module can supply five-servo peak current. Measure voltage under incremental load. Do not assume it is adequate.
5. `begin()` attaches the servo channels but sends no explicit 90-degree command. Some servo libraries may emit their default pulse after attach; therefore first test with horns removed.
6. Before motion, manually place the arm at the angles you will enter in `SYNC`. `SYNC` only stores an open-loop reference; it does not measure physical servo position.

## Build / upload
Install VS Code + PlatformIO extension, open this folder, then:
- Build: `pio run`
- Upload: `pio run -t upload`
- Serial monitor: `pio device monitor -b 115200`
If upload fails, check the COM port and boot mode; reduce `upload_speed` to 115200 if needed.

## Serial commands
- `H`
- `STATUS`
- `TIME,1800` (100..15000 ms)
- `SYNC,90,90,90,90,90` (adopt current physical pose; no motion)
- `J,90,90,90,90,90` (servo command angles)
- `IK,10,0,15,90,90` (XYZ cm, wrist-roll servo angle, gripper servo angle)
- `IKU,x,y,z,rollServo,gripperServo`
- `IKD,x,y,z,rollServo,gripperServo`
- `STOP`

`IK` controls XYZ using base/shoulder/elbow. Wrist roll and gripper are passed as servo command angles. `J` is direct servo-angle mode. New moves are rejected while a move is active; use `STOP` first.

## Validation status
This source package has not been built with PlatformIO or uploaded to the actual ESP32 in this environment. Physical servo calibration, power validation, wrist-axis confirmation, and collision checks remain mandatory. Do not treat this README as evidence of hardware validation.
