# Car1AutoV2Base.java — line by line

**File:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Car1AutoV2Base.java` (244 lines)
**What it is:** the shared engine of both autos. It is `abstract` — it never appears on the Driver Station itself. It handles: hardware init, the INIT menu (delay / ball count / red-blue / small-space test), the shooting phase, closed-loop driving through waypoints using the Pinpoint, glitch-filtered odometry, timing safety, and a log file on the Hub. The two subclasses (`Car1AutoShoot`, `Car1AutoPark`) only choose the path.

---

## Lines 1–18 — package and imports

```java
package org.firstinspires.ftc.teamcode;
```
- **Line 1** — Team package.

```java
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.io.FileWriter;
import java.util.Locale;
```
- **Line 3** — `GoBildaPinpointDriver` — the driver class that talks to the goBILDA Pinpoint odometry computer over I2C.
- **Line 4** — `LynxModule` — represents a REV Control/Expansion Hub; used to switch on *bulk caching* (faster hardware reads).
- **Line 5** — `LinearOpMode` — the "write-your-own-loop" op mode style (as opposed to `OpMode` with separate `init()`/`loop()` methods).
- **Lines 6–8** — motor classes: `DcMotor` (constants like `RunMode`), `DcMotorEx` (the extended motor with velocity control), `DcMotorSimple` (the direction constant FORWARD/REVERSE).
- **Line 9** — `PIDFCoefficients` — a small bag of 4 numbers (P, I, D, F) for the motor's internal speed controller.
- **Line 10** — `Servo` — servo class.
- **Line 11** — `ElapsedTime` — a stopwatch (start at 0, ask `seconds()` later).
- **Lines 13–15** — units and pose types: `AngleUnit` (RADIANS/DEGREES), `DistanceUnit` (MM etc.), `Pose2D` (a position + heading in one object).
- **Lines 17–18** — `FileWriter` (log file), `Locale` (number formatting).

## Lines 20–37 — header comment

- **Lines 20–23** — what this class is, who uses it, and the point-symmetry argument for one-program-two-alliances.
- **Lines 25–29** — the INIT menu controls (explained again below at lines 101–123).
- **Lines 31–32** — the coordinate convention: **start pose = origin, +x = robot front, +y = robot left, counter-clockwise angle +, millimetres**.
- **Lines 34–36** — the honest status: waypoints estimated from the drawing; gate pulse and shot gap untested; the drive gains were tuned on a wooden floor, not field tiles.

## Line 38 — the class declaration

```java
public abstract class Car1AutoV2Base extends LinearOpMode {
```
- **Line 38** — `abstract` = incomplete on purpose; Java refuses to run it directly, so it never shows on the Driver Station. Extending `LinearOpMode` is what makes it an op mode at all: the FTC system calls your `runOpMode()` and you write it like a script, top to bottom.

## Lines 40–43 — drive / Pinpoint constants

```java
    // ---------------- Drive / Pinpoint (same as Car1TeleOp and PP Test; 11-12 PP Test on robot: directions correct) ----------------
    static final boolean LEFT_REVERSED = true, RIGHT_REVERSED = false;
```
- **Line 40** — Section banner comment (groups the constants for humans).
- **Line 41** — Which side of the drive needs its motors reversed so that "positive power = robot forward". The two left motors are mirrored copies of the two right ones, so one side needs REVERSE. Values verified on the robot 11-12 with the PP Test program. Two constants declared in one statement (comma) — just a compact style.

```java
    static final double MAX_P = 0.4, MIN_P = 0.08, KP_XY = 0.004, KP_H = 0.9;
```
- **Line 42** — The four numbers of the steering controller:
  - `MAX_P = 0.4` — motor power never exceeds 40 % while auto-driving (deliberately gentle).
  - `MIN_P = 0.08` — when moving, power is never below 8 % (below that, motors stall and the robot would never reach the target).
  - `KP_XY = 0.004` — proportional gain for distance: power = 0.004 × error-in-mm, so a 250 mm error → 1.0 → capped to 0.4.
  - `KP_H = 0.9` — proportional gain for heading: power = 0.9 × error-in-radians, so a 0.3 rad (17°) error → 0.27 power.

```java
    static final double TOL_MM = 25, TOL_DEG = 3, SEG_TIMEOUT_S = 6;
```
- **Line 43** — "Reached" means within 25 mm and 3° of the target. If a leg takes longer than 6 s, give up on it and go to the next (so one bad leg can't eat the whole auto).

```java
    static final double AUTO_END_S = 29.0;   // keep 1 second spare
```
- **Line 44** — The autonomous period is 30 s; stop driving at 29 s to leave a safety margin.

## Lines 46–58 — shooting constants

```java
    static final double TICKS_PER_REV = 28;
```
- **Line 47** — The flywheel motor's encoder counts 28 ticks per revolution of its shaft. Needed to convert between rpm (human units) and ticks-per-second (what the motor controller wants).

```java
    static final double POLLEN_RPM = 5500;          // 11-12 test shots on robot: front of robot against the wall, POLLEN at 5500 mostly goes in
```
- **Line 48** — The flywheel speed for POLLEN balls, found by real test shots on 11-12.

```java
    static final boolean POLLEN_REVERSED = false;
```
- **Line 49** — The POLLEN flywheel motor spins the correct direction as wired, so no software reversal.

```java
    static final double FLY_P = 10, FLY_I = 1, FLY_D = 0, FLY_F = 32767.0 / 2800;
```
- **Line 50** — The velocity PIDF for the flywheel. **F (feed-forward)** is the workhorse: 32767 is the full-power number of the REV motor controller chip, and 2800 ticks/s is about the flywheel's top speed, so F alone commands "the power that would hold a given speed"; **P = 10** and **I = 1** then correct whatever F gets wrong. Untuned but functional.

```java
    static final double AT_SPEED_BAND = 0.03;
```
- **Line 51** — "At speed" = within 3 % of the target.

```java
    static final double SPINUP_MAX_S = 3.0;         // wait at most 3 seconds to get to speed; shoot anyway if it doesn't
```
- **Line 52** — Don't wait longer than 3 s for the flywheel to reach speed — points for parking beat points for a perfect shot.

```java
    static final long GATE_PULSE_MS = 350;          // how long the gate opens to let out one ball (not tested)
```
- **Line 53** — Each shot = open the gate servo for 350 ms. Milliseconds because `sleep()` takes ms.

```java
    static final double SHOT_GAP_MAX_S = 1.5;       // max wait between two shots to get back to speed
```
- **Line 54** — After each shot the wheel slows; wait for it to recover, but at most 1.5 s.

```java
    static final double SHOOT_DEADLINE_S = 20.0;    // if not done shooting after 20 seconds, stop shooting and go park
```
- **Line 55** — Absolute budget for the whole shooting phase (measured from PLAY, including the start delay).

```java
    static final double GATE_R_OPEN = 0.46, GATE_R_CLOSED = 0.56;   // default values; if car1_servo.txt on the Hub has saved numbers, those are used
```
- **Line 56** — Built-in defaults for the right gate servo; values saved by Hardware Test override them at runtime (via `ServoStore`).

```java
    static final String LOG = "/sdcard/FIRST/car1_auto.txt";
```
- **Line 58** — Path of this auto's log file on the Hub.

## Lines 60–68 — fields (each op mode run gets one copy of these)

```java
    protected DcMotorEx lf, lb, rf, rb, flyP;
```
- **Line 60** — The four drive motors and the POLLEN flywheel. `protected` = subclasses may look at them (they never need to).

```java
    protected Servo gateR;
```
- **Line 61** — The right gate servo (POLLEN path). The auto doesn't use the left gate.

```java
    protected GoBildaPinpointDriver pp;
```
- **Line 62** — The Pinpoint odometry device.

```java
    protected double delayS = 0;
```
- **Line 63** — Chosen start delay in seconds (INIT menu).

```java
    protected int balls = 4;
```
- **Line 64** — How many balls to shoot; default 4.

```java
    protected boolean red = true, smallTest = false;
```
- **Line 65** — Alliance display flag (default red) and the small-space-test switch (default off).

```java
    protected final ElapsedTime match = new ElapsedTime();
```
- **Line 66** — The match clock. `final` = the stopwatch object itself is fixed, but you can reset it. Created now, reset at PLAY.

```java
    private Pose2D good = null;
    private int badReads = 0;
```
- **Lines 67–68** — Glitch filter state: `good` = the last trustworthy position; `badReads` = how many readings were thrown away (shown on screen — a rising count means a wiring problem).

## Lines 70–73 — the abstract questions

```java
    /** Subclass: waypoints {x, y, angle deg}, all in mm on the REAL field. */
    protected abstract double[][] path();
    protected abstract boolean shoots();
    protected abstract String title();
```
- **Line 70** — Javadoc defining the waypoint format for subclasses.
- **Line 71** — `abstract` with no body: every subclass MUST answer "what are your waypoints?".
- **Line 72** — …and "do you shoot?".
- **Line 73** — …and "what's your title line?".

## Lines 75–143 — runOpMode(): the whole auto, top to bottom

```java
    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
```
- **Lines 75–77** — The FTC system calls this once when the op mode starts. First move: for **every** Hub on the robot, turn on bulk caching with AUTO mode — motor/servo data is fetched in one big transaction per loop instead of one per call, which makes the loop much faster (important, because this loop polls the Pinpoint every ~20 ms).

```java
        lf = motor("lf", LEFT_REVERSED); lb = motor("lb", LEFT_REVERSED);
        rf = motor("rf", RIGHT_REVERSED); rb = motor("rb", RIGHT_REVERSED);
```
- **Lines 78–79** — Look up each drive motor **by config name** (must match `car1.xml`), set its direction, and configure it — all inside the `motor()` helper at line 223. If a motor is missing from the config, `motor()` returns `null` instead of crashing (the null-safe `setP()` later skips it).

```java
        if (shoots()) {
            try {
                flyP = hardwareMap.get(DcMotorEx.class, "flywheelPollen");
                flyP.setDirection(POLLEN_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
                flyP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                flyP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                flyP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                flyP.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(FLY_P, FLY_I, FLY_D, FLY_F));
            } catch (Exception e) { flyP = null; }
            try { gateR = hardwareMap.get(Servo.class, "sorterGateR"); } catch (Exception e) gateR = null; }
        } else balls = 0;
```
- **Line 80** — Only grab shooting hardware when the subclass actually shoots (Park Only never touches the Expansion Hub).
- **Line 82** — Get the POLLEN flywheel motor; throws if not in the config.
- **Line 83** — Direction: `POLLEN_REVERSED ? REVERSE : FORWARD` — pick based on the constant (false here → FORWARD).
- **Line 84** — `STOP_AND_RESET_ENCODER` — park the encoder counter at 0.
- **Line 85** — `RUN_USING_ENCODER` — enable the motor controller's closed-loop **velocity** mode; `setVelocity()` then holds a ticks-per-second target.
- **Line 86** — `FLOAT` — when power is 0 the motor coasts (a flywheel braking hard at 5500 rpm is hard on gears).
- **Line 87** — Load the flywheel PIDF numbers from line 50 into the controller.
- **Line 88** — Any failure → `flyP = null`; the auto still runs (and shoots nothing).
- **Line 89** — Same defensive grab for the right gate servo.
- **Line 90** — If this auto doesn't shoot, zero the ball count so the menu doesn't show a meaningless option.

```java
        try {
            pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
            pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            pp.setOffsets(15, 55, DistanceUnit.MM);
            pp.resetPosAndIMU();
        } catch (Exception e) { pp = null; }
```
- **Line 92** — Get the Pinpoint by name `"pp"`.
- **Line 93** — Tell it which pods are attached: the goBILDA 4-bar pod (this sets ticks-per-mm).
- **Line 94** — Both pods count **REVERSED** — the direction correction verified on the robot 11-12 by the PP Test program.
- **Line 95** — Mounting offsets: the X pod is 15 mm left of the robot centre, the Y pod 55 mm in front (needed so turning doesn't fake translation).
- **Line 96** — `resetPosAndIMU()` — position (0,0), heading 0, **right now**. This is why the robot must not be moved after INIT.
- **Line 97** — Any failure → `pp = null` (line 127 then logs and ends safely).

```java
        ServoStore.load();
        if (gateR != null) gateR.setPosition(gateClosed());   // gate closed at the start (holds the preloaded balls)
```
- **Line 98** — Read the saved servo numbers file into memory.
- **Line 99** — Push the gate to its **closed** position immediately, so the preloaded balls can't roll out. `gateClosed()` (line 164) = saved value or the line-56 default.

```java
        boolean pu = false, pd = false, pl = false, pr = false, px = false, py = false;
```
- **Line 101** — "Previous" button states for **edge detection**: comparing current vs previous tells you the moment a button *went down*, not just that it's held (held buttons would add 1 to the delay 50× per second).

```java
        while (opModeInInit()) {
```
- **Line 102** — `opModeInInit()` is true between pressing INIT and pressing PLAY (or STOP). This loop is the INIT menu.

```java
            if (pp != null) pp.update();
```
- **Line 103** — Poll the Pinpoint every cycle — it needs constant `update()` calls to refresh its position data.

```java
            if (gamepad1.dpad_up && !pu) delayS = Math.min(10, delayS + 1);
            if (gamepad1.dpad_down && !pd) delayS = Math.max(0, delayS - 1);
            if (shoots() && gamepad1.dpad_right && !pr) balls = Math.min(4, balls + 1);
            if (shoots() && gamepad1.dpad_left && !pl) balls = Math.max(0, balls - 1);
            if (gamepad1.x && !px) red = !red;
            if (gamepad1.y && !py) smallTest = !smallTest;
```
- **Line 104** — dpad-up *press* → delay +1 s, capped at 10.
- **Line 105** — dpad-down press → delay −1 s, floored at 0.
- **Line 106** — dpad-right press → one more ball, capped at 4 (only in shooting autos).
- **Line 107** — dpad-left press → one fewer ball, floored at 0.
- **Line 108** — X press → flip alliance (display only).
- **Line 109** — Y press → flip the small-space-test mode.

```java
            pu = gamepad1.dpad_up; pd = gamepad1.dpad_down; pl = gamepad1.dpad_left; pr = gamepad1.dpad_right; px = gamepad1.x; py = gamepad1.y;
```
- **Line 110** — Remember this cycle's button states — they become "previous" on the next cycle.

```java
            telemetry.addLine(title());
            telemetry.addData("Alliance", "%s  (X to switch, display only)", red ? "RED" : "BLUE");
            telemetry.addData("Start delay", "%.0f s  (dpad up / down)", delayS);
            if (shoots()) telemetry.addData("Balls to shoot", "%d  (dpad right / left)", balls);
            telemetry.addData("Mode", smallTest ? "small-space test run: path 15%; flywheel off (Y to switch)" : "real match (Y for small-space test run)");
            telemetry.addData("pp", pp == null ? "not found!" : pp.getDeviceStatus().toString());
            if (shoots()) telemetry.addData("POLLEN flywheel / right gate", "%s / %s  (gate open %.2f closed %.2f)", flyP == null ? "not found!" : "OK",
                    gateR == null ? "not found!" : "OK", gateOpen(), gateClosed());
            telemetry.addLine("Once the robot is placed, don't move it. Press PLAY");
            telemetry.update();
```
- **Line 112** — Print the subclass's placement reminder.
- **Line 113** — Alliance + the hint that X switches it.
- **Line 114** — Chosen delay.
- **Line 115** — Chosen ball count (shooting autos only).
- **Line 116** — Current mode: 15 % path with flywheel off, or real match.
- **Line 117** — Pinpoint status (the driver has enum values like `READY` / `NOT_READY`); "not found!" if missing.
- **Lines 118–119** — Flywheel/gate presence plus the open/closed numbers that will be used (from `ServoStore`, defaults shown if nothing saved).
- **Line 120** — The don't-move-the-robot warning.
- **Line 121** — `telemetry.update()` sends everything collected above to the Driver Station screen (telemetry is "collect then send"; without update() nothing appears).

```java
            sleep(30);
```
- **Line 122** — Sleep 30 ms so the loop runs ~33× per second — fast enough for menus, gentle on the CPU.

```java
        }
        if (!opModeIsActive()) return;
```
- **Line 124** — Loop exited because STOP was pressed during INIT (not PLAY)? Then end quietly without touching motors.

```java
        match.reset();
```
- **Line 125** — Start the match clock at the moment PLAY is pressed; every deadline below is measured from here.

```java
        log(String.format(Locale.US, "==== %s %s wait %.0f s shoot %d %s", new java.util.Date(), title(), delayS, balls, smallTest ? "small-space test run" : "real"));
```
- **Line 126** — First log line: timestamp + settings, so a log file from a match can be identified later. `Locale.US` again avoids comma-decimal surprises.

```java
        if (pp == null) { log("pp not found, not moving"); return; }
```
- **Line 127** — Safety gate: without odometry this auto cannot drive — log it and end rather than drive blind.

```java
        while (opModeIsActive() && match.seconds() < delayS) { readPose(); idle(); }
```
- **Line 129** — The chosen start delay: wait, but keep polling the Pinpoint (it must keep being updated) and call `idle()` — which tells the system "I'm just waiting", keeping the STOP button responsive and the thread scheduler happy.

```java
        if (shoots() && balls > 0) shoot();
        if (flyP != null) flyP.setPower(0);   // coast to stop
```
- **Line 131** — The shooting phase (only when the auto shoots and balls were selected).
- **Line 132** — Afterwards, cut flywheel power so it coasts (`FLOAT` from line 86) while driving.

```java
        double k = smallTest ? 0.15 : 1.0;
        for (double[] w : path()) {
            if (!opModeIsActive() || match.seconds() > AUTO_END_S) break;
            boolean ok = goTo(w[0] * k, w[1] * k, w[2]);
            log(String.format(Locale.US, "to (%.0f, %.0f, %.0f deg) %s  %.1f s", w[0] * k, w[1] * k, w[2], ok ? "reached" : "NOT reached", match.seconds()));
        }
```
- **Line 134** — Path scale: 15 % for the small-space test, 100 % for a match. (Note only x and y are scaled — the heading target `w[2]` is passed through unchanged, which is correct: angles don't shrink.)
- **Line 135** — Walk through the subclass's waypoint list one at a time.
- **Line 136** — Stop early if the op mode was stopped or the 29 s budget is gone.
- **Line 137** — Drive to this waypoint (the closed-loop method, line 167). Returns whether it got there within tolerance.
- **Line 138** — Log every leg with its result and the time — the number-one debugging tool after a run ("leg 3 NOT reached at 14.2 s" tells you exactly where to look).

```java
        stopDrive();
        log(String.format(Locale.US, "end %.1f s, position %s, bad reads %d", match.seconds(), poseStr(readPose()), badReads));
        while (opModeIsActive()) { readPose(); telemetry.addData("Done", "%.1f s  %s", match.seconds(), poseStr(good)); telemetry.update(); }
```
- **Line 140** — All legs done (or given up): motors to zero.
- **Line 141** — Final log line: where the robot actually ended up and the glitch-filter's reject count.
- **Line 142** — Then idle in place showing the final position until the referees/driver stops the program (drive motors are already off; Pinpoint keeps updating so you can push the robot and read where it is).

## Lines 146–164 — the shooting phase

```java
    private void shoot() {
        double tps = POLLEN_RPM * TICKS_PER_REV / 60;
```
- **Line 147** — Convert the human target (5500 rpm) into controller units (ticks per second): rpm × 28 ticks / 60 seconds = 2567 tps.

```java
        if (flyP != null && !smallTest) flyP.setVelocity(tps);
```
- **Line 148** — Command that velocity — unless this is a small-space test, where the flywheel must NOT spin (servos still move so you can watch the choreography indoors).

```java
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < SPINUP_MAX_S && !smallTest && !atSpeed(tps)) { readPose(); status("speeding up"); }
```
- **Lines 149–150** — Wait for the wheel to reach speed: stop early if the op mode is stopped, 3 s max, skipped entirely in small-test mode. Odometry keeps updating meanwhile.

```java
        for (int i = 1; i <= balls && opModeIsActive(); i++) {
            if (match.seconds() > SHOOT_DEADLINE_S) { log("past 20 seconds, skipping the rest of the shots"); break; }
            if (gateR != null) gateR.setPosition(gateOpen());
            sleep(GATE_PULSE_MS);
            if (gateR != null) gateR.setPosition(gateClosed());
            log(String.format(Locale.US, "shot %d at %.1f s, flywheel %.0f rpm", i, match.seconds(), flyP == null ? 0 : flyP.getVelocity() * 60 / TICKS_PER_REV));
            t.reset();
            while (opModeIsActive() && t.seconds() < SHOT_GAP_MAX_S && (smallTest ? t.seconds() < 0.8 : !atSpeed(tps) || t.seconds() < 0.4)) { readPose(); status("after shot " + i + ", getting back to speed"); }
        }
```
- **Line 151** — One iteration = one shot, counting 1..balls.
- **Line 152** — The 20 s budget check: past it, log and stop shooting (parking points wait for no one).
- **Line 153** — Open the gate (saved open value from `ServoStore`, or default).
- **Line 154** — Hold it open 350 ms — the untested "one ball" pulse time.
- **Line 155** — Close it again.
- **Line 156** — Log the shot with the flywheel's actual speed at that moment (converted back to rpm for humans; `flyP == null ? 0 :` guards the missing-motor case).
- **Line 157** — Restart the stopwatch for the recovery wait.
- **Line 158** — Wait before the next shot: at most 1.5 s; in real mode, until at-speed **or** 0.4 s minimum gap (the `||` means both "not yet at speed" and "under 0.4 s" keep the loop going); in small-test mode simply 0.8 s. Odometry keeps updating during the wait.

```java
    private boolean atSpeed(double tps) { return flyP == null || Math.abs(flyP.getVelocity() - tps) <= AT_SPEED_BAND * tps; }
```
- **Line 162** — Speed check: within ±3 % of target. If the motor is missing entirely, treat it as "at speed" so a hardware failure can't stall the auto forever.

```java
    private double gateOpen() { return ServoStore.get("sorterGateR", true, GATE_R_OPEN); }
    private double gateClosed() { return ServoStore.get("sorterGateR", false, GATE_R_CLOSED); }
```
- **Lines 163–164** — Thin wrappers: the open/closed positions, preferring numbers saved by Hardware Test over the code defaults.

## Lines 167–186 — goTo(): closed-loop driving to one waypoint

```java
    private boolean goTo(double tx, double ty, double tdeg) {
```
- **Line 167** — Takes a target x, y (mm) and heading (degrees); returns true if reached.

```java
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < SEG_TIMEOUT_S && match.seconds() < AUTO_END_S) {
```
- **Lines 168–169** — Steering loop: keep going while the program runs, this leg is under 6 s, and the match is under 29 s.

```java
            Pose2D p = readPose();
            double cx = p.getX(DistanceUnit.MM), cy = p.getY(DistanceUnit.MM), ch = p.getHeading(AngleUnit.RADIANS);
```
- **Lines 170–171** — Where am I? Current x, y in mm and heading in radians (glitch-filtered by `readPose()`).

```java
            double ex = tx - cx, ey = ty - cy, eh = wrap(Math.toRadians(tdeg) - ch);
```
- **Line 172** — The three **errors**: how far I still need to go in x and y, and the heading error (target − current), wrapped to −180°…+180° so +179° vs −179° correctly reads as a 2° error, not −358°.

```java
            if (Math.hypot(ex, ey) < TOL_MM && Math.abs(Math.toDegrees(eh)) < TOL_DEG) { stopDrive(); return true; }
```
- **Line 173** — Arrived? Straight-line distance (`hypot` = Pythagoras) under 25 mm **and** heading within 3° → stop and report success.

```java
            double fwd = Math.cos(ch) * ex + Math.sin(ch) * ey;
            double left = -Math.sin(ch) * ex + Math.cos(ch) * ey;
```
- **Lines 174–175** — Rotate the field-frame error into the **robot's frame**: "to reach the target I must move this much *forward* and this much *to my left*". These two lines are a standard 2-D rotation matrix; `ch` is the robot's current heading.

```java
            // Scale both by the same amount, so the direction stays the same when driving diagonally
            double f = KP_XY * fwd, l = KP_XY * left, m = Math.hypot(f, l);
            if (m > MAX_P) { f *= MAX_P / m; l *= MAX_P / m; }
            else if (m > 0.001 && m < MIN_P) { f *= MIN_P / m; l *= MIN_P / m; }
```
- **Line 176** — Comment explaining the subtlety below.
- **Line 177** — Proportional power for each axis, and `m` = the combined magnitude.
- **Line 178** — Too strong? Shrink **both by the same factor** → the robot slows down but keeps pointing the same direction (clamping each axis separately would bend diagonal paths).
- **Line 179** — Too weak to move? Boost both by the same factor to the 8 % floor (skipped when essentially zero, so the robot can fully stop).

```java
            double r = Math.max(-MAX_P, Math.min(MAX_P, KP_H * eh));
```
- **Line 180** — Turn power = 0.9 × heading error (radians), clamped between ±0.4.

```java
            drive(f, -l, -r);
```
- **Line 181** — Send to the wheels. The sign flips convert this method's conventions (forward+, left+, counter-clockwise+) into `drive()`'s (forward+, right+, clockwise+) — see line 215.

```java
            status(String.format(Locale.US, "going to (%.0f, %.0f)  error %.0f mm", tx, ty, Math.hypot(ex, ey)));
```
- **Line 182** — Live progress on the Driver Station.

```java
        }
        stopDrive();
        return false;
```
- **Lines 183–185** — Loop ended without reaching (timeout / match end / stop): halt and report failure — the caller logs it and continues with the next leg.

## Lines 188–192 — status(): the during-drive screen

```java
    private void status(String s) {
        telemetry.addData("Auto", "%s  %.1f s", s, match.seconds());
        telemetry.addData("pp", "%s  bad reads %d", poseStr(good), badReads);
        telemetry.update();
    }
```
- **Line 189** — Current action + match time.
- **Line 190** — Position and glitch count (if `bad reads` climbs, suspect a pod/cable problem).
- **Line 191** — Send to the screen.

## Lines 195–207 — readPose(): odometry with a glitch filter

```java
    protected Pose2D readPose() {
        pp.update();
        Pose2D p = pp.getPosition();
```
- **Line 196** — Poll the Pinpoint (must happen every cycle).
- **Line 197** — Get its cached position.

```java
        boolean bad = Double.isNaN(p.getX(DistanceUnit.MM)) || Double.isNaN(p.getY(DistanceUnit.MM)) || Double.isNaN(p.getHeading(AngleUnit.RADIANS));
```
- **Line 198** — A reading of `NaN` ("not a number") means the device gave garbage — mark it bad.

```java
        if (!bad && good != null) {
            double j = Math.hypot(p.getX(DistanceUnit.MM) - good.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM) - good.getY(DistanceUnit.MM));
            double jh = Math.abs(Math.toDegrees(wrap(p.getHeading(AngleUnit.RADIANS) - good.getHeading(AngleUnit.RADIANS))));
            if (j > 60 || jh > 15) bad = true;
        }
```
- **Lines 199–203** — Jump check: if the position "jumped" more than 60 mm or the heading more than 15° since the last good reading — physically impossible in one 20 ms loop — the reading is a glitch; throw it away. (This exact bug bit the team on 11-12: a jump tripped the PP Test's safety stop; the fix is documented in Car1PPTest.java lines 204–208.)

```java
        if (bad) { badReads++; return good == null ? p : good; }
        good = p;
        return p;
    }
```
- **Line 204** — Bad reading: count it, and return the last good one instead (never return a lie).
- **Lines 205–206** — Good reading: remember it as the new "last good" and return it.

## Lines 209–212 — poseStr(): position as text

```java
    private static String poseStr(Pose2D p) {
        if (p == null) return "-";
        return String.format(Locale.US, "x %.0f y %.0f  %.1f deg", p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM), p.getHeading(AngleUnit.DEGREES));
    }
```
- **Line 210** — Null guard ("-" before the first reading).
- **Line 211** — Round-trip-free formatting: x, y, heading.

## Lines 215–221 — drive() and stopDrive(): the mecanum mix

```java
    private void drive(double y, double x, double r) {
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        setP(lf, (y + x + r) / d); setP(lb, (y - x + r) / d);
        setP(rf, (y - x - r) / d); setP(rb, (y + x - r) / d);
    }
```
- **Line 215** — Conventions: `y` = forward+, `x` = strafe right+, `r` = turn clockwise+.
- **Line 216** — Normalisation: `d` is the sum of absolute values, but at least 1 — so any command bigger than "full" gets scaled down to fit, while small commands pass through unchanged.
- **Lines 217–218** — The classic mecanum wheel mix. Check it yourself: `y=1` alone → all four get 1 (forward). `x=1` alone → lf=+1, lb=−1, rf=−1, rb=+1 (the alternating diagonal pattern that makes a mecanum strafe). `r=1` alone → left pair +1, right pair −1 (spin clockwise).

```java
    private void stopDrive() { setP(lf, 0); setP(lb, 0); setP(rf, 0); setP(rb, 0); }
```
- **Line 221** — All drive power to 0 (null-safe via `setP`).

## Lines 223–233 — helpers

```java
    private DcMotorEx motor(String n, boolean rev) {
        try {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, n);
            m.setDirection(rev ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return m;
        } catch (Exception e) { return null; }
    }
```
- **Line 226** — Look up the motor by config name.
- **Line 227** — Apply the direction fix.
- **Line 228** — `RUN_WITHOUT_ENCODER` — plain power control. The drive wheels' own encoders aren't used at all; **position comes from the Pinpoint's dead wheels**, so this is the right mode.
- **Line 229** — `BRAKE` — zero power short-circuits the motor so it stops crisply (matters when holding a waypoint).
- **Line 230** — Missing motor → `null`, never a crash.

```java
    private static void setP(DcMotorEx m, double p) { if (m != null) m.setPower(p); }
```
- **Line 233** — Null-safe "set power": every call site can ignore whether the motor exists.

## Lines 235–239 — wrap(): keep angles in range

```java
    protected static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
```
- **Lines 236–238** — Add or subtract full turns (2π) until the angle lands in (−π, +π]. This makes angle *differences* meaningful — see line 172.

## Lines 241–243 — log(): append to the Hub file

```java
    protected void log(String s) {
        try (FileWriter w = new FileWriter(LOG, true)) { w.write(s + "\n"); } catch (Exception ignored) { }
    }
```
- **Line 242** — Open the auto log in **append** mode (`true`), write the line plus newline, auto-close; if writing fails, ignore silently (logging must never break the auto).

---

## Evaluation

**Strengths**
- Clean template-method design: both autos share 100 % of the hard logic; subclasses are data, not code.
- Defensive everywhere: every device lookup can fail without crashing; missing odometry is detected before moving.
- Real timing discipline: 29 s global budget, 6 s per leg, 20 s shooting deadline, 3 s spin-up cap — the auto cannot eat the clock.
- The waypoint controller is a textbook P-controller with two thoughtful touches: joint min/max scaling (preserves direction on diagonals) and a wrap-aware heading error.
- The glitch filter came from a real diagnosed incident (documented in Car1PPTest) — evidence the code is *response to the robot*, not guesswork.
- Everything is logged; every leg reports reached/NOT-reached with timestamps.

**Watch-outs (not changed)**
- **Autos never ran on the robot** (per README): first runs should use the Y small-space mode, then a full-size dry run on a real field.
- `shoot()` commands full velocity instantly (`setVelocity` on line 148) — TeleOp deliberately ramps over ≥1 s "to protect the printed couplers"; the auto does not ramp. If that protection matters, it matters here too.
- P-only steering with a 0.4 power cap: expect steady following error on carpet (the robot tracks *toward* waypoints rather than exactly on the line between them) — fine for parking, but don't expect mm-precision.
- `GATE_PULSE_MS = 350` is explicitly marked untested; too short = no ball drops, too long = two balls.
- Heading target is always 0 in both paths, so the heading loop only *holds* heading — good; but it also means a bumped robot during INIT ruins the whole frame (warned on screen).
