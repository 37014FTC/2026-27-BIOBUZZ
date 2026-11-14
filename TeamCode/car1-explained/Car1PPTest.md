# Car1PPTest.java — line by line

**File:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Car1PPTest.java` (292 lines)
**What it is:** a guided self-test for the Pinpoint odometry in a 1 m × 1 m space. It nudges the robot in six known directions, judges whether the odometry agreed, tells you in plain words what's wrong if it didn't ("X is backwards — press X to flip"), and — only if all six pass — runs a closed-loop demo of the same waypoint-driving math the autos use. Every result is appended to a log file on the Hub.
**Why it matters:** this is the program that produced the "X and Y both REVERSED" fact baked into every other file, and it's where the bad-read glitch filter (copied into the auto base) was discovered on 11-12.

---

## Lines 1–17 — package and imports

```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.io.FileWriter;
import java.util.Locale;
```
- **Line 1** — Team package.
- **Line 3** — The Pinpoint driver class.
- **Line 4** — `LynxModule` for bulk caching.
- **Lines 5–6** — LinearOpMode + the `@TeleOp` annotation.
- **Lines 7–9** — Motor classes, direction constant, and `ElapsedTime`.
- **Lines 11–13** — Units + `Pose2D`.
- **Lines 16–17** — Log-file writing + Locale-safe formatting.
- (No `Servo` import — this program never touches servos.)

## Lines 19–35 — header comment

- **Lines 20–23** — Purpose and the ground rules: 1 m × 1 m of clear floor, each step moves ≤ ~20 cm, automatic stop past 35 cm, results logged to `/sdcard/FIRST/car1_pptest.txt`.
- **Lines 25–29** — Controls: A = next step, B = abort, X/Y = flip the X/Y pod direction (then it restarts from step 1), back = re-zero.
- **Lines 31–33** — The 7 steps: six open-loop nudges (forward/back/left/right/turn-left/turn-right), then the closed-loop combo.
- **Line 34** — The Pinpoint's coordinate convention (x forward+, y left+, counter-clockwise+).

## Lines 36–37 — annotation and class

```java
@TeleOp(name = "Car1 PP Test", group = "Test")
public class Car1PPTest extends LinearOpMode {
```
- **Line 36** — Registered under group "Test" alongside the Hardware Test.
- **Line 37** — LinearOpMode again.

## Lines 39–43 — constants

```java
    static final boolean LEFT_REVERSED = true, RIGHT_REVERSED = false;   // same as Car1TeleOp
```
- **Line 39** — Drive-motor direction constants — deliberately identical to TeleOp and the auto base (the comment says so), so the wheel behaviour in this test matches the real programs.

```java
    static final double OPEN_POWER = 0.25, OPEN_TURN = 0.25, OPEN_DIST_MM = 150, OPEN_ANGLE_DEG = 45, OPEN_TIME_S = 0.8;
```
- **Line 40** — The open-loop steps' numbers: drive at 25 % power, turn at 25 %, stop after 150 mm of travel or 45° of turn or 0.8 s — whichever comes first. Small, slow, safe.

```java
    static final double MAX_RADIUS_MM = 350;
```
- **Line 41** — The hard safety fence: if the robot ever gets more than 35 cm from where the Pinpoint was last zeroed, stop immediately. This is what protects your table/walls if something is very wrong.

```java
    static final double KP_XY = 0.004, KP_H = 0.9, MAX_P = 0.3, MIN_P = 0.08, TOL_MM = 15, TOL_DEG = 2, STEP_TIMEOUT_S = 3;
```
- **Line 42** — The closed-loop step-7 gains — the same values the auto uses (with a tighter 0.3 power cap, tighter tolerances 15 mm / 2°, and a 3 s per-move timeout because the moves are tiny).

```java
    static final String LOG = "/sdcard/FIRST/car1_pptest.txt";
```
- **Line 43** — This program's log file.

## Lines 45–52 — state

```java
    private DcMotorEx lf, lb, rf, rb;
```
- **Line 45** — The four drive motors.

```java
    private GoBildaPinpointDriver pp;
```
- **Line 46** — The Pinpoint.

```java
    private boolean revX = true, revY = true;   // what the programs use now: both REVERSED (checked on robot 11-12)
```
- **Line 47** — Current pod-direction settings; starts at the known-good values (both reversed) so a healthy robot passes without pressing anything.

```java
    private int step = 1;
```
- **Line 48** — Which step is next (1–7; 8 = all done).

```java
    private final String[] result = new String[8];
```
- **Line 49** — One summary line per step; index 0 unused so index = step number (a small memory trade for readable code).

```java
    private boolean dirX = false, dirY = false, dirH = false;
```
- **Line 50** — The three pass/fail flags: X direction OK, Y direction OK, heading OK. All three must be true before step 7 will run.

```java
    private String last = "";
```
- **Line 51** — The most recent message to show on screen.

```java
    private boolean pA, pB, pX, pY, pBack;
```
- **Line 52** — Previous-cycle button states (edge detection, same pattern as everywhere else in this codebase).

## Lines 54–87 — runOpMode(): setup, INIT screen, main loop

```java
    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        lf = motor("lf", LEFT_REVERSED); lb = motor("lb", LEFT_REVERSED);
        rf = motor("rf", RIGHT_REVERSED); rb = motor("rb", RIGHT_REVERSED);
```
- **Line 56** — Bulk caching on every Hub.
- **Lines 57–58** — The four motors via the `motor()` helper (line 248) — direction set, encoder reset then RUN_WITHOUT_ENCODER, BRAKE stop, `null` if missing.

```java
        try {
            pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
            pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pp.setOffsets(15, 55, DistanceUnit.MM);   // numbers from the old chassis; only affects x/y while turning, not the direction check here
            applyDirs();
        } catch (Exception e) { pp = null; }
```
- **Line 60** — Get the Pinpoint.
- **Line 61** — 4-bar pods.
- **Line 62** — Mounting offsets, with the comment explaining why an imprecise offset doesn't invalidate this test: offsets only distort position *while turning*, and the direction checks (steps 1–6) barely turn.
- **Line 63** — `applyDirs()` (line 95) pushes the revX/revY flags into the device and re-zeros it.
- **Line 64** — Failure → `null`.

```java
        log("==== " + new java.util.Date() + " start, pp " + (pp == null ? "not found" : pp.getDeviceStatus()));
```
- **Line 65** — Start a new log section with a timestamp.

```java
        while (opModeInInit()) {
            if (pp != null) pp.update();
            telemetry.addLine("Car1 PP Test: put robot in the middle of open floor, 50 cm clear all around. Press PLAY, then A for each step, B to stop any time");
            telemetry.addData("pp", pp == null ? "not found! check which I2C port pp is on in config car1" : pp.getDeviceStatus().toString());
            if (pp != null) telemetry.addData("First push the robot by hand", "push forward: x goes up, push left: y goes up, turn left: angle goes up -> %s", pose());
            telemetry.update();
            sleep(50);
        }
```
- **Line 67** — The INIT phase loop.
- **Line 68** — Keep polling the Pinpoint.
- **Line 69** — Setup instructions.
- **Line 70** — Pinpoint status / where to check if missing.
- **Line 71** — The manual pre-check: push the robot by hand before PLAY and watch live x/y/angle — the zeroth test, done by hand, before any motor turns.
- **Lines 72–73** — Send the screen, pace the loop at ~20 Hz.

```java
        if (pp == null) return;
```
- **Line 75** — No Pinpoint → nothing to test. End.

```java
        while (opModeIsActive()) {
            pp.update();
            if (gamepad1.x && !pX) { revX = !revX; applyDirs(); restart("Flipped X pod direction"); }
            if (gamepad1.y && !pY) { revY = !revY; applyDirs(); restart("Flipped Y pod direction"); }
            if (gamepad1.back && !pBack) { pp.resetPosAndIMU(); sleep(300); restart("Reset"); }
            if (gamepad1.a && !pA) runStep();
            pA = gamepad1.a; pB = gamepad1.b; pX = gamepad1.x; pY = gamepad1.y; pBack = gamepad1.back;
            show();
        }
        stopDrive();
```
- **Line 77** — The main loop: no movement happens here — only on button presses.
- **Line 78** — Poll the Pinpoint every cycle (it must be read constantly to stay fresh).
- **Line 79** — X press → flip the X-pod direction, apply it, wipe all results, restart from step 1 (a direction change invalidates every previous verdict).
- **Line 80** — Y press → same for the Y pod.
- **Line 81** — Back press → re-zero the Pinpoint, wait 300 ms for the IMU to settle, restart from step 1.
- **Line 82** — A press → run the next step (the only thing that moves the robot).
- **Line 83** — Update edge-detection memory.
- **Line 84** — Redraw the screen.
- **Line 86** — STOP pressed → motors off.

## Lines 89–93 — restart(): wipe and begin again

```java
    private void restart(String why) {
        step = 1; for (int i = 0; i < result.length; i++) result[i] = null;
        dirX = dirY = dirH = false; last = why + ", restart from step 1";
        log(why + " -> X pod " + (revX ? "REVERSED" : "FORWARD") + ", Y pod " + (revY ? "REVERSED" : "FORWARD"));
    }
```
- **Line 90** — Back to step 1 and clear every stored result line.
- **Line 91** — Clear the three pass flags (they must be re-earned under the new settings); tell the user why.
- **Line 92** — Log the change with the full new direction state — so a results file always says which settings produced it.

## Lines 95–100 — applyDirs(): push flags into the device

```java
    private void applyDirs() {
        pp.setEncoderDirections(revX ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                revY ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pp.resetPosAndIMU();
        sleep(300);
    }
```
- **Lines 96–97** — Convert the two booleans into driver enum values and set them.
- **Line 98** — Re-zero (the coordinate frame just changed; old positions are meaningless).
- **Line 99** — 300 ms pause for the reset to take and the IMU to settle.

## Lines 103–117 — runStep(): the step dispatcher

```java
    private void runStep() {
        switch (step) {
            case 1: openLoop(1, "forward", OPEN_POWER, 0, 0); break;
            case 2: openLoop(2, "back", -OPEN_POWER, 0, 0); break;
            case 3: openLoop(3, "left", 0, -OPEN_POWER * 1.2, 0); break;
            case 4: openLoop(4, "right", 0, OPEN_POWER * 1.2, 0); break;
            case 5: openLoop(5, "turn left", 0, 0, -OPEN_TURN); break;
            case 6: openLoop(6, "turn right", 0, 0, OPEN_TURN); break;
            case 7:
                if (!(dirX && dirY && dirH)) { last = "Steps 1-6 directions not all correct, closed loop will not run (first flip with X / Y, or send the results to the team)"; return; }
                closedLoop(); break;
            default: last = "All steps done. Results saved on the Hub in car1_pptest.txt"; return;
        }
        step++;
    }
```
- **Line 103** — Called on each A press.
- **Lines 104–110** — Steps 1–6: open-loop nudges. Note the sign conventions in the arguments (`fwd` forward+, `right` right+, `cw` clockwise+ — per the Javadoc at line 119): forward is +; left is negative *right*; turn **left** is negative *cw*.
- Strafe values are ×1.2 (mecanum strafe weaker than forward at the same power — the same fact TeleOp handles with `STRAFE_FIX`).
- **Lines 111–113** — Step 7 is gated: only runs when steps 1/3/5 all passed their direction check; otherwise an explanatory message and no movement.
- **Line 114** — Step ≥ 8: everything done; just remind where results live.
- **Line 116** — Advance the step counter — reached only for steps that actually ran.

## Lines 119–149 — openLoop(): one measured nudge

```java
    /** fwd forward +, right right +, cw clockwise +. Stops at 15 cm / 45 deg or 0.8 seconds. */
```
- **Line 119** — Javadoc: the sign conventions and the stop conditions.

```java
    private void openLoop(int k, String name, double fwd, double right, double cw) {
        good = null;
        Pose2D p0 = readPose();
        int[] e0 = enc();
```
- **Line 121** — Reset the glitch filter's "last good" (a fresh start of measurement).
- **Line 122** — Snapshot the starting position (the move is measured as a *difference*, so it doesn't matter where the robot is).
- **Line 123** — Snapshot all four drive encoder counts (to check wheel directions alongside odometry).

```java
        ElapsedTime t = new ElapsedTime();
        String stopWhy = "time up 0.8 s";
```
- **Line 124** — Stopwatch for the move.
- **Line 125** — Default reason for stopping (overwritten if a different condition fires first) — the *reason* gets logged, not just the fact.

```java
        while (opModeIsActive()) {
            if (gamepad1.b) { stopWhy = "B pressed"; break; }
            Pose2D p = readPose();
            double dx = p.getX(DistanceUnit.MM) - p0.getX(DistanceUnit.MM), dy = p.getY(DistanceUnit.MM) - p0.getY(DistanceUnit.MM);
            double dh = Math.toDegrees(wrap(p.getHeading(AngleUnit.RADIANS) - p0.getHeading(AngleUnit.RADIANS)));
            if (cw == 0 && Math.hypot(dx, dy) >= OPEN_DIST_MM) { stopWhy = "pp moved 15 cm"; break; }
            if (cw != 0 && Math.abs(dh) >= OPEN_ANGLE_DEG) { stopWhy = "pp turned 45 deg"; break; }
            if (radius(p) > MAX_RADIUS_MM) { stopWhy = "more than 35 cm from start, safety stop"; break; }
            if (t.seconds() > OPEN_TIME_S) break;
            drive(fwd, right, cw);
            telemetry.addData("Running", "%d %s  %.1f s", k, name, t.seconds()); telemetry.update();
        }
        stopDrive();
```
- **Line 126** — Move until one of five stop conditions.
- **Line 127** — B pressed → abort (user override).
- **Line 128** — Current (filtered) position each cycle.
- **Line 129** — Movement so far, as x/y differences from the start.
- **Line 130** — Heading change so far, in degrees, wrap-safe.
- **Line 131** — Straight-line move ≥ 150 mm → enough data, stop.
- **Line 132** — Turn ≥ 45° → enough, stop.
- **Line 133** — Distance from the zero point > 350 mm → the safety fence (runaway protection).
- **Line 134** — 0.8 s → time's up (also covers "odometry says nothing is moving").
- **Line 135** — Command the motion (the mecanum mix, line 233).
- **Line 136** — Live progress line on screen.

```java
        settle(400);
        Pose2D p = readPose();
        int[] e = enc();
        double dx = p.getX(DistanceUnit.MM) - p0.getX(DistanceUnit.MM), dy = p.getY(DistanceUnit.MM) - p0.getY(DistanceUnit.MM);
        double dh = Math.toDegrees(wrap(p.getHeading(AngleUnit.RADIANS) - p0.getHeading(AngleUnit.RADIANS)));
```
- **Line 139** — Wait 400 ms while *continuously reading* the Pinpoint — the robot coasts a little after power off, and if you didn't read during the coast, the jump would look exactly like a glitch and get filtered out (the `settle` Javadoc at line 212 explains this).
- **Lines 140–143** — Final measurements: position, encoder counts, and the total movement deltas.

```java
        String verdict = judge(k, dx, dy, dh);
        result[k] = String.format(Locale.US, "%d %s: dx %+.0f dy %+.0f mm  turn %+.1f deg  wheels lf %+d lb %+d rf %+d rb %+d  (%s) -> %s",
                k, name, dx, dy, dh, e[0] - e0[0], e[1] - e0[1], e[2] - e0[2], e[3] - e0[3], stopWhy, verdict);
        last = result[k];
        log(result[k]);
```
- **Line 144** — Get the human verdict (next method).
- **Lines 145–146** — Build the one-line summary: what the odometry saw (dx, dy, turn), what the wheels did (encoder deltas — sign shows direction), why it stopped, and the verdict. `%+d` prints the sign (+123/−123) — essential for reading direction from a log.
- **Line 147** — Show it as the "last event" on screen.
- **Line 148** — Append to the Hub log.

## Lines 151–161 — judge(): the brain of the test

```java
    private String judge(int k, double dx, double dy, double dh) {
        double ax = Math.abs(dx), ay = Math.abs(dy);
        boolean moved = Math.hypot(dx, dy) > 30 || Math.abs(dh) > 10;
        if (!moved) return "pp barely changed (robot didn't move? or pp pods not touching the floor / cable not plugged in)";
```
- **Line 152** — Absolute movements (direction of the *bigger* axis is what matters).
- **Line 153** — Did anything measurable happen at all? (>30 mm or >10°.)
- **Line 154** — No → the classic dead-pod symptoms, named outright so a rookie knows what to check.

```java
        switch (k) {
            case 1: if (ax > ay && dx > 0) { dirX = true; return "OK"; } return ax > ay ? "X is backwards (press X to flip)" : "x / y swapped? robot moved forward but pp y is changing";
            case 3: if (ay > ax && dy > 0) { dirX = true; return "OK"; } return ay > ax ? "Y is backwards (press Y to flip)" : "x / y swapped? robot moved left but pp x is changing";
            case 5: if (dh > 0) { dirX = true; return "OK (turning left makes angle go up)"; } return "angle is backwards (Pinpoint mounted upside down? tell the team)";
            default: return "return trip";
        }
```
- **Line 156** — Step 1 (drove **forward**): correct means the change was mostly in **x** (`ax > ay`) and **positive**. If mostly-x but negative → the X pod is reversed → literally tells you which button to press. If mostly-y → the pods are plugged into each other's ports.
- **Line 157** — Step 3 (drove **left**): same logic for y — mostly-y and positive = OK; mostly-y and negative = reversed; mostly-x = swapped.
- **Line 158** — Step 5 (turned **left**): heading must increase; if it decreased, the Pinpoint itself is mounted upside-down — not fixable with a button, hence "tell the team".
- **Line 159** — Steps 2/4/6 are the return legs (they retrace the previous move), so they carry no direction verdict.
- (Each OK branch also sets its `dir*` flag — the gate for step 7.)

## Lines 163–195 — closedLoop(): step 7, the mini-auto

```java
        double[][] tg = {{200, 0, 0}, {0, 0, 0}, {0, 200, 0}, {0, 0, 0}, {0, 0, 90}, {0, 0, 0}};
```
- **Line 164** — Six mini-waypoints: forward 200 mm, back, left 200 mm, back, turn left 90°, turn back.

```java
        good = null;
        int bad0 = badReads;
        Pose2D o = readPose();
        double ox = o.getX(DistanceUnit.MM), oy = o.getY(DistanceUnit.MM), oh = o.getHeading(AngleUnit.RADIANS);
```
- **Line 165** — Reset the glitch filter.
- **Line 166** — Remember the bad-read count at the start (to report how many occurred *during* the test).
- **Lines 167–168** — Capture the origin: everything below is measured **relative to where the robot sits now**, so the test works wherever you placed it.

```java
        StringBuilder sb = new StringBuilder("7 closed loop:");
        for (double[] g : tg) {
            ElapsedTime t = new ElapsedTime();
            double ex = 0, ey = 0, eh = 0;
            boolean ok = false;
```
- **Line 169** — The log/report accumulator.
- **Line 170** — Loop over the six waypoints.
- **Lines 171–174** — Per waypoint: a stopwatch, error variables (pre-set for the report if the loop never runs), and the reached flag.

```java
            while (opModeIsActive() && !gamepad1.b && t.seconds() < STEP_TIMEOUT_S) {
                Pose2D p = readPose();
                // Use the position where the closed loop started as the origin (pp coordinates)
                double cx = p.getX(DistanceUnit.MM) - ox, cy = p.getY(DistanceUnit.MM) - oy, ch = wrap(p.getHeading(AngleUnit.RADIANS) - oh);
                if (Math.hypot(cx, cy) > MAX_RADIUS_MM) { sb.append(" [over 35 cm safety stop]"); break; }
                ex = g[0] - cx; ey = g[1] - cy; eh = wrap(Math.toRadians(g[2]) - ch);
                if (Math.hypot(ex, ey) < TOL_MM && Math.abs(Math.toDegrees(eh)) < TOL_DEG) { ok = true; break; }
                double fwd = Math.cos(ch) * ex + Math.sin(ch) * ey;    // robot coordinates: forward
                double left = -Math.sin(ch) * ex + Math.cos(ch) * ey;  // robot coordinates: left
                double f = clipMag(KP_XY * fwd), l = clipMag(KP_XY * left), r = clipMag(KP_H * eh);
                drive(f, -l, -r);
                telemetry.addData("Closed loop", "target (%.0f, %.0f, %.0f deg)  error %.0f mm %.1f deg  %.1f s", g[0], g[1], g[2], Math.hypot(ex, ey), Math.toDegrees(eh), t.seconds());
                telemetry.update();
            }
            stopDrive(); settle(300);
```
- **Line 174** — Drive while: program alive, B not pressed, under 3 s.
- **Line 175** — Filtered pose.
- **Line 177** — Subtract the origin — current position *relative to test start*.
- **Line 178** — Safety fence still active.
- **Line 179** — Errors = target − current (heading target converted to radians first).
- **Line 180** — Reached = within 15 mm and 2°.
- **Lines 181–182** — Rotate errors into robot forward/left — the same two rotation-matrix lines as the auto's `goTo()`.
- **Line 183** — Proportional powers via `clipMag` (line 197).
- **Line 184** — Convert forward/left/ccw to forward/right/cw for `drive()`.
- **Lines 185–186** — Progress on screen.
- **Line 188** — Stop and let the robot settle (reading odometry throughout).

```java
            sb.append(String.format(Locale.US, "  (%.0f,%.0f,%.0f deg) %s error %.0f mm %.1f deg %.1f s;", g[0], g[1], g[2], ok ? "reached" : "NOT reached", Math.hypot(ex, ey), Math.toDegrees(eh), t.seconds()));
            if (gamepad1.b) sb.append(" [B pressed]");
            if (gamepad1.b || !ok) break;
        }
        sb.append(String.format(Locale.US, "  bad reads %d", badReads - bad0));
        result[7] = sb.toString(); last = result[7]; log(result[7]);
```
- **Line 189** — Append this waypoint's outcome to the report.
- **Lines 190–191** — Stop the whole sequence on B or on a missed waypoint (no point continuing if tracking failed).
- **Line 193** — Total bad reads during the test — the health number for the Pinpoint wiring.
- **Line 194** — Store, show, and log the report.

## Lines 197–201 — clipMag()

```java
    private double clipMag(double v) {
        double a = Math.min(MAX_P, Math.abs(v));
        if (a > 0.001) a = Math.max(MIN_P, a);
        return Math.copySign(a, v);
    }
```
- **Line 198** — Cap the magnitude at 0.3.
- **Line 199** — If it's not essentially zero, lift it to at least 0.08 (the " motors don't move below this" floor).
- **Line 200** — Re-attach the original sign. Note: this clips each axis *independently*, unlike the auto's joint scaling — the two controllers differ slightly (see Evaluation).

## Lines 204–216 — the glitch filter (and its origin story)

```java
    // ---------------- Read pp (filter out bad reads) ----------------
    // 11-12 test: the second closed-loop run stopped after 0.4 s. The log showed the position reading suddenly jumped
    // a lot and set off the 35 cm safety stop.
    // In one loop (about 20 ms) the robot can't move 60 mm or turn 15 deg, so anything bigger is a bad read: throw it
    // away and use the last good value.
    private Pose2D good = null;
    private int badReads = 0;
```
- **Lines 205–208** — The comment block: a real incident on 11-12, its symptom (safety stop after 0.4 s), its cause (a position jump), and the physical reasoning behind the thresholds (20 ms ⇒ ≤60 mm / ≤15° is possible; more is impossible ⇒ bad read).
- **Lines 209–210** — The filter state.

```java
    /** Stop and wait for the robot to coast to a stop, reading pp the whole time (if we don't read, the coasting distance would look like a bad read). */
    private void settle(long ms) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.milliseconds() < ms) { readPose(); sleep(10); }
    }
```
- **Line 212** — Javadoc explaining the subtle reason `settle()` exists: during coasting the robot moves without new commands; if you stopped reading odometry, the accumulated coast distance would exceed 60 mm and the filter would eat real data.
- **Lines 213–215** — Wait the given ms, reading the Pinpoint every 10 ms.

## Lines 218–230 — readPose(): the filter itself

```java
    private Pose2D readPose() {
        pp.update();
        Pose2D p = pp.getPosition();
        boolean bad = Double.isNaN(p.getX(DistanceUnit.MM)) || Double.isNaN(p.getY(DistanceUnit.MM)) || Double.isNaN(p.getHeading(AngleUnit.RADIANS));
        if (!bad && good != null) {
            double j = Math.hypot(p.getX(DistanceUnit.MM) - good.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM) - good.getY(DistanceUnit.MM));
            double jh = Math.abs(Math.toDegrees(wrap(p.getHeading(AngleUnit.RADIANS) - good.getHeading(AngleUnit.RADIANS))));
            if (j > 60 || jh > 15) bad = true;
        }
        if (bad) { badReads++; return good == null ? p : good; }
        good = p;
        return p;
    }
```
- **Line 219** — Poll the device.
- **Line 220** — Get the position.
- **Line 221** — NaN in any field = garbage.
- **Lines 222–226** — Jump test vs the last good reading: >60 mm or >15° in one cycle = physically impossible = bad.
- **Line 227** — Bad: count it; return the last good pose instead (or the raw one if we've never had a good one).
- **Lines 228–229** — Good: remember and return it.

## Lines 232–246 — drive / stopDrive / enc / motor / setP

```java
    private void drive(double y, double x, double r) {
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        setP(lf, (y + x + r) / d); setP(lb, (y - x + r) / d);
        setP(rf, (y - x - r) / d); setP(rb, (y + x - r) / d);
    }
```
- **Lines 233–237** — The same mecanum mix as everywhere: y forward+, x right+, r clockwise+; normalise by the sum of magnitudes (min 1).

```java
    private void stopDrive() { setP(lf, 0); setP(lb, 0); setP(rf, 0); setP(rb, 0); }
```
- **Line 239** — All stop.

```java
    private int[] enc() {
        DcMotorEx[] m = {lf, lb, rf, rb};
        int[] e = new int[4];
        for (int i = 0; i < 4; i++) e[i] = m[i] == null ? 0 : m[i].getCurrentPosition();
        return e;
    }
```
- **Lines 241–246** — Snapshot the four drive encoder counts (0 for missing motors) — used to print wheel directions next to odometry deltas in each result line.

```java
    private DcMotorEx motor(String n, boolean rev) { ... }
    private static void setP(DcMotorEx m, double p) { if (m != null) m.setPower(p); }
```
- **Lines 248–257** — Motor lookup/config and null-safe setPower — identical helpers to the auto base.

## Lines 262–291 — show(), pose(), radius(), wrap(), log()

```java
    private void show() {
        String[] names = {"", "forward", "back", "left", "right", "turn left", "turn right", "closed loop (fwd 20 / back / left 20 / back / turn left 90 / back)"};
        telemetry.addData("Next step (press A)", step <= 7 ? step + " " + names[step] : "all done");
```
- **Lines 263–264** — Step names by index (index 0 unused) and the "what's next" line — the operator never needs to remember the list.

```java
        telemetry.addData("pp", "%s  %s", pp.getDeviceStatus(), pose());
        telemetry.addData("Bad reads (thrown away)", badReads);
        telemetry.addData("pp pod directions", "X %s | Y %s  (X / Y buttons flip)", revX ? "REVERSED" : "FORWARD", revY ? "REVERSED" : "FORWARD");
        telemetry.addLine(last);
        for (int i = 1; i <= 7; i++) if (result[i] != null) telemetry.addLine(result[i]);
```
- **Line 265** — Live status + pose.
- **Line 266** — The glitch counter — a healthy setup reads 0.
- **Line 267** — Current pod direction settings + which buttons flip them.
- **Line 268** — The latest event message.
- **Line 269** — Every result collected so far, in order — the whole session's history stays on one screen.

```java
        String miss = (lf == null ? "lf " : "") + (lb == null ? "lb " : "") + (rf == null ? "rf " : "") + (rb == null ? "rb " : "");
        if (!miss.isEmpty()) telemetry.addData("Motors not found", miss);
        telemetry.addLine("B stop | back reset and restart");
        telemetry.update();
    }
```
- **Lines 270–271** — Name-and-shame any missing drive motors.
- **Lines 272–273** — Footer hint + send.

```java
    private String pose() {
        Pose2D p = pp.getPosition();
        return String.format(Locale.US, "x %.0f  y %.0f mm  angle %.1f deg", p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM), p.getHeading(AngleUnit.DEGREES));
    }
```
- **Lines 276–279** — Pose formatted as one string (used by the INIT screen and `show()`).

```java
    private static double radius(Pose2D p) { return Math.hypot(p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM)); }
```
- **Line 281** — Distance from the zero point — the safety-fence calculation.

```java
    private static double wrap(double a) { ... }
```
- **Lines 283–287** — Fold any angle into (−π, π] — same helper as the auto.

```java
    private void log(String s) {
        try (FileWriter w = new FileWriter(LOG, true)) { w.write(s + "\n"); } catch (Exception ignored) { }
    }
```
- **Lines 289–291** — Append-and-never-fail logging.

---

## Evaluation

**Strengths**
- This is the most pedagogically valuable file in the zip: it turns "my odometry is wrong" into a checklist with plain-English verdicts and the exact button to fix each failure — including the swapped-pods and upside-down cases that a direction flag *can't* fix.
- Diagnosis is data-driven: every step logs odometry deltas **and** wheel encoder deltas, so one log line distinguishes "wheel wrong" from "pod wrong".
- Safety is layered: 25 % power, 150 mm stop, 0.8 s stop, 35 cm hard fence, B abort — you can run this on a desk edge.
- The glitch filter and `settle()` show real engineering: an incident (11-12 safety-stop at 0.4 s) was root-caused from logs and turned into a general guard that now protects the autos too.

**Watch-outs (not changed)**
- Step 7 uses per-axis clipping (`clipMag`) while the auto uses joint scaling — same gains, slightly different diagonal behaviour. Both work; just don't be surprised by small differences between "PP Test step 7" tracking and auto tracking.
- The pass flags (`dirX/dirY/dirH`) are earned only by steps 1/3/5 in one run; flipping a pod mid-session correctly wipes them, but if you skip a step (never press A on step 1) step 7 stays locked — by design, but worth knowing.
- The judge's thresholds (30 mm / "mostly x vs y") are coarse on purpose; a wildly miscalibrated pod scale would still pass direction checks — but that's not what this test is for.
