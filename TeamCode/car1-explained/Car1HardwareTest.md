# Car1HardwareTest.java — line by line

**File:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Car1HardwareTest.java` (192 lines)
**What it is:** a bench-test program: pick any one motor or servo with the dpad and drive it alone, watch every encoder live on the Driver Station, and — the key feature — **save** tuned servo open/closed numbers to the Hub file that TeleOp and the autos read (`ServoStore`). It was the tool used to find the reversed motors and the gate servo values recorded in the other files' comments.

---

## Lines 1–13 — package and imports

```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
```
- **Line 1** — Team package (also why `ServoStore` needs no import).
- **Line 3** — Pinpoint driver class.
- **Lines 4–5** — `LinearOpMode` base class + the `@TeleOp` annotation class.
- **Lines 6–8** — Motor classes and the direction constant.
- **Line 9** — Servo class.
- **Lines 11–13** — Angle/distance units and the `Pose2D` position type (for the Pinpoint readout at the bottom of the screen).

## Lines 15–32 — header comment

- **Lines 16–17** — Purpose: one device at a time, config names matching the wiring table.
- **Lines 19–30** — The full control map: dpad up/down selects a device; B stops everything; stick runs motors; X/Y/A run flywheels; bumpers nudge servos ±0.02; Y/A also *save* open/closed; back resets the Pinpoint.
- **Lines 29–30** — What the screen shows; and the design rule: a missing device never crashes, it shows "not found".

## Lines 33–34 — annotation and class

```java
@TeleOp(name = "Car1 Hardware Test", group = "Test")
public class Car1HardwareTest extends LinearOpMode {
```
- **Line 33** — Registers as a TeleOp (group "Test", so it sorts separately from match programs). No `preselectTeleOp` needed — it's never followed by a match.
- **Line 34** — A `LinearOpMode`: the whole life of the program is the `runOpMode()` script below.

## Lines 36–50 — device tables and state

```java
    private static final String[] MOTORS = {"lf", "lb", "rf", "rb", "flywheelNectar", "flywheelPollen", "intake"};
```
- **Line 36** — The motor config names **in selection order**: indices 0–3 drive, 4 = NECTAR flywheel, 5 = POLLEN flywheel, 6 = intake. Being a `String[]` table means adding a motor = adding a name here.

```java
    private static final String[] SERVOS = {"sorterGateL", "sorterGateR", "sorterEndL", "sorterEndR"};
```
- **Line 37** — The four servo names, continuing the same numbering after the motors.

```java
    private static final int N_MOTORS = MOTORS.length;
    private static final int N_DEVICES = MOTORS.length + SERVOS.length;
```
- **Lines 38–39** — 7 motors; 11 devices total. Used for the `sel` wrap-around and for deciding "is the selection a motor or a servo".

```java
    private final DcMotorEx[] motor = new DcMotorEx[N_MOTORS];
    private final Servo[] servo = new Servo[SERVOS.length];
    private final double[] servoPos = new double[SERVOS.length];
    private final boolean[] servoTouched = new boolean[SERVOS.length];
```
- **Line 41** — Parallel array: `motor[i]` is the object for `MOTORS[i]` (or `null` if not found).
- **Line 42** — Same for servos.
- **Line 43** — The servo's current commanded position (starts at 0.5 = centre).
- **Line 44** — "Has this servo been commanded yet?" — used for the don't-move-until-touched safety and for the screen ("not moved yet").

```java
    private final double[] flyPower = {0.2, 0.2};
    private final boolean[] flyOn = {false, false};
```
- **Lines 45–46** — Per-flywheel state: last chosen power (starts at a safe 0.2) and on/off, indexed 0 = NECTAR, 1 = POLLEN.

```java
    private GoBildaPinpointDriver pp;
```
- **Line 47** — The Pinpoint, shown read-only at the bottom of the screen.

```java
    private int sel = 0;
    private boolean pUp, pDown, pX, pY, pA, pB, pLB, pRB, pBack;
```
- **Line 49** — Which device is selected (index into the combined list; 0 = `lf` at start).
- **Line 50** — Previous-cycle button states for edge detection (press vs hold) — one flag per button.

## Lines 52–95 — runOpMode(): setup then the control loop

```java
    @Override
    public void runOpMode() {
        for (int i = 0; i < N_MOTORS; i++) {
            try {
                motor[i] = hardwareMap.get(DcMotorEx.class, MOTORS[i]);
                motor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor[i].setZeroPowerBehavior(i < 4 ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
            } catch (Exception e) {
                motor[i] = null;
            }
        }
```
- **Line 54** — Loop over all 7 motors.
- **Line 56** — Look up by name; throws if missing.
- **Line 57** — Zero the encoder counter.
- **Line 58** — Plain power mode (this program is about *observing* encoders, not closed loop).
- **Line 59** — Stop behaviour: drive motors (i < 4) **brake**, the other three **coast** — flywheels especially should coast.
- **Lines 60–62** — Missing motor → `null`, program continues (screen shows "not found").

```java
        // Wiring table section 5: the two left drive motors start as REVERSE (we guess the two sides are mirror images).
        // If a wheel spins the wrong way, change it here, don't swap wires
        setDir("lf", DcMotorSimple.Direction.REVERSE);
        setDir("lb", DcMotorSimple.Direction.REVERSE);
        setDir("intake", DcMotorSimple.Direction.REVERSE);   // 11-11 tested on robot: intake was backwards (same as Car1TeleOp)
        setDir("flywheelNectar", DcMotorSimple.Direction.REVERSE);   // 11-11 tested on robot: NECTAR flywheel was backwards
```
- **Lines 64–65** — Comment: the standing instruction to fix direction in *code*, not by swapping wires (swapping wires also swaps the encoder polarity).
- **Line 66** — Left front reversed (mirror-image guess).
- **Line 67** — Left back reversed.
- **Line 68** — Intake reversed — *found with this very program on 11-11*.
- **Line 69** — NECTAR flywheel reversed — same.

```java
        for (int i = 0; i < SERVOS.length; i++) {
            try {
                servo[i] = hardwareMap.get(Servo.class, SERVOS[i]);
            } catch (Exception e) {
                servo[i] = null;
            }
            servoPos[i] = 0.5;
        }
```
- **Lines 71–78** — Same pattern for the servos; note there is **no `setPosition` here** — a servo receives no signal until you select it (line 110), so a wrong saved number can't slam it at startup. Position variable starts at 0.5.

```java
        try {
            pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
            // 11-03: 4-bar odometry pods, X pod 15 mm left of center, Y pod 55 mm in front of center,
            // X / Y both REVERSED (tested on the old chassis; on 11-12 checked again on the new chassis with Car1 PP Test: still correct)
            pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                    GoBildaPinpointDriver.EncoderDirection.REVERSED);
            pp.setOffsets(15, 55, DistanceUnit.MM);   // if this line gives a compile error, delete ", DistanceUnit.MM" (old drivers only take mm)
            pp.resetPosAndIMU();
        } catch (Exception e) {
            pp = null;
        }
```
- **Line 81** — Get the Pinpoint.
- **Lines 82–83** — Comment recording the mounting facts and their provenance (measured 11-03, re-verified 11-12).
- **Line 84** — Pod type (sets mm-per-tick).
- **Lines 85–86** — Both pods reversed.
- **Line 87** — Mounting offsets, **plus a note about a possible compile error on older SDKs** where `setOffsets` only took two numbers (this project's SDK compiles it fine with the unit).
- **Line 88** — Zero position and IMU.
- **Lines 89–91** — Failure → `null`.

```java
        telemetry.addLine("Car1 Hardware Test ready. Press PLAY to start. dpad up/down = next device, B = stop all");
        telemetry.update();
        waitForStart();
```
- **Lines 93–95** — One banner line, send it, and wait for PLAY (`waitForStart()` handles the INIT phase by itself — that's the LinearOpMode convenience).

## Lines 97–129 — the main loop

```java
        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            if (up && !pUp) changeSel(-1);
            if (down && !pDown) changeSel(+1);
```
- **Line 97** — Runs until STOP.
- **Line 98** — Snapshot the dpad states (read gamepad once per cycle).
- **Line 99** — dpad-up **press** (edge) → previous device.
- **Line 100** — dpad-down press → next device.

```java
            if (gamepad1.b && !pB) stopAll();
```
- **Line 102** — B press → everything off (the panic button).

```java
            if (sel < N_MOTORS) {
                if (sel == 4 || sel == 5) flywheel(sel - 4);
                else if (motor[sel] != null) motor[sel].setPower(-gamepad1.left_stick_y);
            } else {
```
- **Line 104** — Is the selection one of the 7 motors?
- **Line 105** — Flywheel slots (4, 5) get their own on/off control (helper at line 131).
- **Line 106** — Other motors: stick directly sets power. `-left_stick_y` because gamepads report stick-up as **negative** y — the minus flips it so pushing up = positive power.

```java
                int s = sel - N_MOTORS;
                if (servo[s] != null) {
                    if (!servoTouched[s]) { servoTouched[s] = true; servo[s].setPosition(servoPos[s]); }
                    if (gamepad1.right_bumper && !pRB) servoPos[s] = clip(servoPos[s] + 0.02, 0, 1);
                    if (gamepad1.left_bumper && !pLB) servoPos[s] = clip(servoPos[s] - 0.02, 0, 1);
                    servo[s].setPosition(servoPos[s]);
                    if (gamepad1.y && !pY) ServoStore.save(SERVOS[s], true, servoPos[s]);
                    if (gamepad1.a && !pA) ServoStore.save(SERVOS[s], false, servoPos[s]);
                }
            }
```
- **Line 108** — Convert the combined index into a servo index (0–3).
- **Line 110** — First time this servo is selected: mark touched and command 0.5 — this is the moment the servo first receives any signal.
- **Line 111** — Right-bumper press: +0.02, kept inside 0…1 by `clip` (line 155).
- **Line 112** — Left-bumper press: −0.02.
- **Line 113** — Continuously command the current value (continuous = easy to read on screen).
- **Line 114** — Y press → **save the current position as this servo's "open"** to `/sdcard/FIRST/car1_servo.txt` (via `ServoStore.save`).
- **Line 115** — A press → save as "closed". These two lines are the whole tuning workflow: nudge with bumpers until it looks right, save with Y or A; TeleOp picks the numbers up next start.

```java
            if (pp != null) {
                if (gamepad1.back && !pBack) pp.resetPosAndIMU();
                pp.update();
            }
```
- **Lines 119–122** — Back press → re-zero the Pinpoint (robot must be still); `pp.update()` every cycle keeps its data fresh for the bottom-of-screen readout.

```java
            pUp = up; pDown = down; pX = gamepad1.x; pY = gamepad1.y; pA = gamepad1.a; pB = gamepad1.b;
            pLB = gamepad1.left_bumper; pRB = gamepad1.right_bumper; pBack = gamepad1.back;
            show();
```
- **Lines 124–125** — This cycle's buttons become next cycle's "previous" — the second half of edge detection.
- **Line 126** — Draw the whole screen (helper at line 157).

```java
        }
        stopAll();
```
- **Line 128** — Loop ended (STOP pressed) → everything off. (Ending with motors stopped is an FTC good-practice rule.)

## Lines 131–137 — flywheel(): the two special motors

```java
    private void flywheel(int k) {
        DcMotorEx m = motor[4 + k];
        if (gamepad1.x && !pX) { flyOn[k] = !flyOn[k]; if (flyOn[k]) flyPower[k] = 0.2; }
        if (gamepad1.y && !pY) flyPower[k] = clip(flyPower[k] + 0.1, 0, 1);
        if (gamepad1.a && !pA) flyPower[k] = clip(flyPower[k] - 0.1, 0, 1);
        if (m != null) m.setPower(flyOn[k] ? flyPower[k] : 0);
    }
```
- **Line 132** — The motor object for flywheel slot k (0 = NECTAR at index 4, 1 = POLLEN at index 5).
- **Line 133** — X press → toggle; every switch-on resets power to 0.2 (so a forgotten 1.0 from last time can't surprise you).
- **Line 134** — Y press → +0.1 power.
- **Line 135** — A press → −0.1.
- **Line 136** — Apply: chosen power if on, else 0.
- Why flywheels aren't stick-driven: you want to set a power and *watch the encoder* — a stick can't be held perfectly still.

## Lines 139–144 — changeSel(): move the selection

```java
    private void changeSel(int d) {
        // The device you leave: stick-controlled motors stop (flywheels keep their state, so you can watch the numbers
        // while switching to another device; press B to stop them)
        if (sel < N_MOTORS && sel != 4 && sel != 5 && motor[sel] != null) motor[sel].setPower(0);
        sel = (sel + d + N_DEVICES) % N_DEVICES;
    }
```
- **Lines 141–142** — Comment: the design decision — leaving a stick-motor stops it (the stick now belongs to another device), but flywheels **keep running** so you can switch to the Pinpoint readout and watch the speed settle; B stops them.
- **Line 142** — Implementing exactly that: stop the old selection only if it's a stick-controlled motor.
- **Line 143** — Move the index with wrap-around: `+ d` (d is −1 or +1), `+ N_DEVICES` first so it can't go negative, then `% N_DEVICES` folds it into 0…10.

## Lines 146–149 — stopAll()

```java
    private void stopAll() {
        for (DcMotorEx m : motor) if (m != null) m.setPower(0);
        flyOn[0] = false; flyOn[1] = false;
    }
```
- **Line 147** — Every motor to 0.
- **Line 148** — And clear the flywheel on/off flags so their state matches reality.

## Lines 151–153 — setDir()

```java
    private void setDir(String name, DcMotorSimple.Direction dir) {
        for (int i = 0; i < N_MOTORS; i++) if (MOTORS[i].equals(name) && motor[i] != null) motor[i].setDirection(dir);
    }
```
- **Line 152** — Find the motor by name in the table and set its direction — a name-based helper so lines 66–69 read like a wiring checklist instead of index arithmetic.

## Line 155 — clip()

```java
    private static double clip(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
```
- **Line 155** — Clamp v into [lo, hi]: first `min` caps the top, then `max` lifts the bottom. Servo positions must never leave 0…1.

## Lines 157–191 — show(): the whole dashboard

```java
    private void show() {
        String name = sel < N_MOTORS ? MOTORS[sel] : SERVOS[sel - N_MOTORS];
        telemetry.addData("> Selected", "%s  (%d/%d)", name, sel + 1, N_DEVICES);
```
- **Line 158** — The selected device's name (from whichever table it's in).
- **Line 159** — Show it with its 1-based position (e.g. `sorterGateR  (8/11)`).

```java
        if (sel < N_MOTORS) {
            DcMotorEx m = motor[sel];
            if (m == null) telemetry.addLine("   Motor not found: check config name / wiring");
            else if (sel == 4 || sel == 5) telemetry.addData("   Flywheel", "%s  power %.1f  (X on/off | Y +0.1 | A -0.1)", flyOn[sel - 4] ? "on" : "off", flyPower[sel - 4]);
            else telemetry.addData("   Power", "%.2f  (left stick up/down)", m.getPower());
        } else {
            int s = sel - N_MOTORS;
            if (servo[s] == null) telemetry.addLine("   Servo not found: check config name / wiring");
            else {
                telemetry.addData("   Servo position", "%.2f  (right bumper +0.02 | left bumper -0.02)", servoPos[s]);
                telemetry.addData("   Saved", "open %s | closed %s  (Y save open | A save closed)", ServoStore.fmt(SERVOS[s], true), ServoStore.fmt(SERVOS[s], false));
                if (ServoStore.lastError != null) telemetry.addData("   Error saving file", ServoStore.lastError);
            }
        }
```
- **Lines 160–164** — Motor selected: "not found" if missing; flywheels show on/off + power + controls; other motors show live power (`getPower()` reads back what was commanded).
- **Lines 165–173** — Servo selected: position + bumpers hint; then the crucial feedback — **what is currently saved** as open/closed (`ServoStore.fmt`, "not saved" when nothing is), so you always see the saved state, not just what you're commanding; plus any file error from the last save.

```java
        telemetry.addLine("-- Motors: encoder / speed (ticks per second) / power");
        for (int i = 0; i < N_MOTORS; i++) {
            if (motor[i] == null) telemetry.addData(MOTORS[i], "not found");
            else telemetry.addData(MOTORS[i], "%d  /  %.0f  /  %.2f", motor[i].getCurrentPosition(), motor[i].getVelocity(), motor[i].getPower());
        }
```
- **Line 174** — Banner for the all-motors table.
- **Lines 175–178** — Every motor, always: encoder count / ticks-per-second / power. This is how you verify "encoder counts up when the wheel spins forward" without selecting anything.

```java
        telemetry.addLine("-- Servos (only shows ones that have moved)");
        for (int i = 0; i < SERVOS.length; i++)
            telemetry.addData(SERVOS[i], servo[i] == null ? "not found" : (servoTouched[i] ? String.format("%.2f", servoPos[i]) : "not moved yet"));
```
- **Lines 179–181** — Every servo: "not found", its position once touched, or "not moved yet" — an honest display of the no-signal-until-selected safety design.

```java
        if (pp == null) telemetry.addData("pp", "not found (Pinpoint config name pp, Control Hub I2C bus 0)");
        else {
            Pose2D p = pp.getPosition();
            telemetry.addData("pp", "x %.1f mm   y %.1f mm   angle %.1f deg   (back to reset)", p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM), p.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pp status", pp.getDeviceStatus());
        }
        telemetry.addLine("B stop all | dpad up/down next device");
        telemetry.update();
    }
```
- **Line 182** — Missing Pinpoint → a hint naming the exact config name and bus to check.
- **Lines 183–188** — Present: live x / y / heading (push the robot by hand and watch — the quickest wiring sanity check) plus device status.
- **Line 189** — Footer cheat-sheet.
- **Line 190** — Send the screen.

---

## Evaluation

**Strengths**
- A genuine tuning workflow, not a toy: the bumper-nudge + Y/A-save loop is how every servo number in the other files was found, and the results persist via `ServoStore`.
- "Don't move any servo until selected" (lines 77, 110) is a real safety decision — a wrong saved value can't crash a gate at startup.
- Full-robot dashboard (all encoders, all servos, odometry) means you can verify wiring in one screen without selecting each device.
- Comments act as a lab notebook: dates, what was wrong, what fixed it ("11-11 tested on robot: intake was backwards").

**Watch-outs (not changed)**
- The screen is rebuilt from scratch ~60×/s — fine on a Driver Station, but the telemetry `String.format` calls do add loop jitter; irrelevant for a test tool.
- `flyPower` resets to 0.2 on every switch-on — if you deliberately want to re-test at a higher power you re-press Y a few times; a small UX tax for safety.
- `stopAll()` doesn't touch servos — by design (servos hold position), but remember gates stay where they were after STOP.
