# Car 1 robot code (12 Nov 2026)

OnBot Java code for Car 1. Package `org.firstinspires.ftc.teamcode`.

## Files

| File | What it is | Shows on Driver Station as |
|---|---|---|
| `TeamCode/Car1TeleOp.java` | Match TeleOp (driver-controlled period) | **Car1 TeleOp** |
| `TeamCode/Car1HardwareTest.java` | Test each motor / servo one at a time; set and save the gate servo open/closed numbers | **Car1 Hardware Test** |
| `TeamCode/Car1PPTest.java` | Check the Pinpoint odometry (pp) and drive wheel directions in a small space (1 m x 1 m) | **Car1 PP Test** |
| `TeamCode/Car1AutoShoot.java` | Main auto: shoot preloaded POLLEN, LEAVE, drive to LOADING ZONE, PARK | **Car1 Auto Shoot+Park** |
| `TeamCode/Car1AutoPark.java` | Backup auto: LEAVE + PARK only | **Car1 Auto Park Only** |
| `TeamCode/Car1AutoV2Base.java` | Helper: shared code for both autos (init menu, shooting, drive-to-point) | (not an OpMode) |
| `TeamCode/ServoStore.java` | Helper: reads / writes the servo open/closed numbers file on the Hub | (not an OpMode) |
| `car1.xml` | Robot configuration file | config "car1" |

## Status

- TeleOp, Hardware Test and PP Test have been **tested on the robot**.
- The two autos are **compiled but NOT yet run on the robot**.
- Auto waypoints are **estimated from the field drawing** and must be checked on a real field.

## Install

1. These are OnBot Java files. Upload the 7 `.java` files to the Control Hub folder
   `/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/`
   (or use the OnBot Java page in the Robot Controller web page), then click **Build Everything**.
2. Copy `car1.xml` to `/sdcard/FIRST/` on the Control Hub, then on the Driver Station go to
   Configure Robot and **activate** the config `car1`.

## Hardware config names

The code only cares about the names. If a device is missing, the program does not crash; the screen shows "not found".

**Control Hub**
- Motors: 0 `lb`, 1 `lf`, 2 `rb`, 3 `rf`
- Servos: 0 `sorterGateL`, 1 `sorterGateR`, 2 `sorterEndL`, 3 `sorterEndR`
- I2C bus 0: `imu` (built in) and `pp` (goBILDA Pinpoint), both on the same bus

**Expansion Hub 2**
- Motors: 0 `flywheelNectar`, 1 `flywheelPollen`, 2 `intake`

## Settings to know

- **Gate servo values** are saved on the Hub in `/sdcard/FIRST/car1_servo.txt` (Hardware Test writes it, TeleOp and Auto read it).
  Current values: left gate open 0.90 / closed 0.78, right gate open 0.46 / closed 0.56.
  If the file is missing, the same numbers are built into the code.
- **Flywheel speeds** are fixed at start: NECTAR (big ball) 3800 rpm, POLLEN (small ball) 5500 rpm.
  Dpad changes them only for that run.
- **Pinpoint pods**: X and Y are both REVERSED (checked on the robot on 11-12 with Car1 PP Test).

## Car1 TeleOp button map

Mechanism buttons work on BOTH gamepads (one person with gamepad1 can test everything; in a match the operator uses gamepad2):

| Button | What it does |
|---|---|
| A | intake on / off |
| B | back up then push: reverse 0.25 s, forward 1 s, then back to the old on/off. Use when a ball is stuck between the intake and the sorter. Does not spit the ball out |
| X | both flywheels on / off (on: ramps up slowly; off: power cut, coast to stop) |
| Y | not used (flywheels have only one speed: NECTAR 3800 / POLLEN 5500 rpm) |
| left bumper (hold) | left gate `sorterGateL` opens, NECTAR (big ball) goes into the flywheel; release to close |
| right bumper (hold) | right gate `sorterGateR` opens, POLLEN (small ball) goes into the flywheel; release to close |
| | If the flywheels are on but that wheel is not at speed yet, the gate waits until at speed (or at most 0.5 s), then stays open until you release. If the flywheels are off, it opens right away (to clear balls) |
| right trigger (hold) | one-button shooting from against the wall: turns flywheels on if off; when both are at speed, both gates open for a short time, close, wait until back at speed, open again... until you release |
| dpad up / down | NECTAR flywheel target +/- 100 rpm (this run only, not saved) |
| dpad right / left | POLLEN flywheel target +/- 100 rpm |

gamepad1 only (driver):

| Button | What it does |
|---|---|
| left stick | drive (forward / back, left / right) |
| right stick left/right | turn on the spot |
| left trigger (hold) | slow mode |
| back | reset heading (for field-centric: press when the front of the robot points away from you) |
| right stick press | switch robot-centric / field-centric (default robot-centric; shown on the first line of the screen) |
| left stick press | switch "back of robot is the front" / normal (robot-centric only; on = 2 buzzes, off = 1 long buzz) |

gamepad2 only (operator):

| Button | What it does |
|---|---|
| left trigger (hold) | both end servos `sorterEndL` / `sorterEndR` open (not fitted on Car 1 yet, so nothing happens) |
| back | stop all mechanisms (intake off, flywheels off) |

Other:
- After turning on the flywheels or changing speed, the gamepads buzz once the first time both flywheels are at speed. The screen shows "AT SPEED - ready to feed".
- Flywheel guard: if a flywheel can't spin up for 1.5 s (encoder unplugged / wrong direction / jammed), power is cut and a warning shows. Press X to restart.
- Gamepads buzz at 1:30 and 1:50 (30 and 10 seconds left).
- Shot log: every shot (speed drop > 5%) is counted on screen and saved to `/sdcard/FIRST/car1_shots.csv` when the program ends.

## How to run the autos

1. Place the robot (see the start position below), select the auto, press **INIT**. After INIT, don't move the robot (the Pinpoint resets to zero).
2. Use the init menu on gamepad1:
   - **dpad up / down**: start delay, 0-10 seconds
   - **dpad right / left**: how many balls to shoot, 0-4 (Shoot+Park only)
   - **X**: red / blue. **Display only**, it does not change the path (the field is symmetric, so the moves are the same)
   - **Y**: small-space test run. The path shrinks to 15% and the flywheel does not spin (servos still move). Use this to test in a small room.
3. Press **PLAY**. After the auto ends, the Driver Station switches to Car1 TeleOp.

Start positions:
- **Car1 Auto Shoot+Park**: front of the robot against the wall with the CELL whose up-facing side is your alliance color, lined up with that CELL, in your own half, not touching the garden or flowers, not in the LOADING ZONE. It shoots POLLEN, backs away from the wall (LEAVE), then drives along the side to the LOADING ZONE (PARK).
- **Car1 Auto Park Only**: back of the robot against your driver-area wall, front facing into the field, the LOADING ZONE on the robot's left (not touching its tape). It drives forward 200 mm (LEAVE), then 550 mm left into the LOADING ZONE (PARK).

Each auto writes a log to `/sdcard/FIRST/car1_auto.txt` on the Hub.
