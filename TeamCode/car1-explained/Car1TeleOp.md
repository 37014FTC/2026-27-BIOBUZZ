# Car1TeleOp.java — line by line

**File:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Car1TeleOp.java` (563 lines)
**What it is:** the match TeleOp (driver-controlled period), version 3, tested on the robot 11-12. Two-station control: gamepad1 drives, gamepad2 runs mechanisms — but every mechanism button works on *both* pads so one person can test everything alone. It includes a slow-ramp flywheel with a stall guard, "at speed" feedback with rumble, gate servos that wait for flywheel speed, one-button burst shooting, an unjam sequence for the intake, a shot detector that logs every shot to a CSV, battery/time warnings, and field-centric / back-is-front drive options.

---

## Lines 1–29 — package and imports

```java
package org.firstinspires.ftc.teamcode;
```
- **Line 1** — Team package.

```java
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;
```
- **Line 3** — Pinpoint driver.
- **Line 4** — Hub module class (bulk caching).
- **Line 5** — `RevHubOrientationOnRobot` — describes which way the Control Hub is physically mounted (needed by its IMU).
- **Lines 6–7** — LinearOpMode + `@TeleOp`.
- **Lines 8–10** — Motor classes + direction constant.
- **Line 11** — `Gamepad` — the class used for the cur/prev snapshot trick (line 161).
- **Line 12** — `IMU` — the Control Hub's built-in inertial measurement unit (the yaw gyro used for field-centric).
- **Line 13** — `PIDFCoefficients` — the 4-number PIDF bag.
- **Line 14** — Servo class.
- **Line 15** — `VoltageSensor` — reads battery voltage.
- **Line 16** — Stopwatch class.
- **Lines 18–19** — Angle and distance units.
- **Lines 21–24** — File reading/writing tools.
- **Line 25** — `SimpleDateFormat` — timestamps for the shot CSV.
- **Lines 26–29** — `ArrayList`/`List` (collect shot log lines in memory), `Date` (timestamps), `Locale` (formatting).

## Lines 31–99 — the header comment

- **Lines 32–37** — Version history (v3 = 11-12), and the config-name philosophy: the program only cares about **names**, not ports; missing devices show "not found" instead of crashing.
- **Lines 39–56** — The full mechanism button map (A intake, B unjam, X flywheels, bumpers = gates, right trigger = burst, dpad = speed trims). Line 49–51 documents the v3 gate behaviour: hold a bumper and the gate *waits* for that flywheel to reach speed, then latches open until release; flywheels off → opens instantly (for clearing balls).
- **Lines 57–64** — The driver-only buttons: left stick = drive, right stick x = turn, left trigger = slow, back = reset heading, right-stick-press = field-centric toggle, left-stick-press = "back is front" toggle.
- **Lines 65–68** — The operator-only buttons: left trigger = end servos (not fitted on Car 1), back = stop all mechanisms.
- **Lines 69–70** — The rumble-once-when-at-speed feature.
- **Lines 72–79** — What v3 added: the shot log, the (currently disabled) saved-flywheel-speeds file, battery/time display, and 1:30 / 1:50 rumble warnings.
- **Lines 81–87** — Safety list: servos don't move until first pressed; flywheels ramp ≥1 s; the 1.5 s stall guard that cuts power.
- **Lines 89–97** — The checklist of robot-dependent constants and how to set each (Hardware Test, PP Test, shooting tests, hub orientation).
- **Line 99** — Honest history note: v1 was compile-only; v3 has been driven.

## Lines 101–102 — annotation and class

```java
@TeleOp(name = "Car1 TeleOp", group = "Car1")
public class Car1TeleOp extends LinearOpMode {
```
- **Line 101** — Registers as **Car1 TeleOp** — the exact name both autos' `preselectTeleOp` points at.
- **Line 102** — LinearOpMode again: the program is one script with one big loop.

## Lines 104–146 — the constants block

### Hub orientation (105–107)

```java
    static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;       // not measured
    static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;    // not measured
    static final boolean USE_PP_HEADING = false;   // true = field-centric uses the Pinpoint heading
```
- **Line 105** — Which way the hub's logo faces on the robot (a guess, "not measured").
- **Line 106** — Which way the USB ports face (also a guess).
- **Line 107** — Because those guesses power the IMU's coordinate fix, there's an escape hatch: set true to take heading from the Pinpoint instead.

### Drive (109–112)

```java
    static final boolean LEFT_REVERSED = true;     // lf lb REVERSE (the two sides are mirror images; a guess)
    static final boolean RIGHT_REVERSED = false;
    static final double STRAFE_FIX = 1.1;          // mecanum strafe correction
    static final double SLOW_SCALE = 0.4;
```
- **Line 109** — Left motors reversed (mirror-image reasoning).
- **Line 110** — Right motors normal.
- **Line 111** — Mecanum robots strafe a bit weaker than they drive; multiplying the sideways stick by 1.1 compensates.
- **Line 112** — Slow mode drives at 40 %.

### Flywheel (114–126)

```java
    static final double TICKS_PER_REV = 28;        // 5203 1:1 motor shaft: 28 ticks per turn
    static final double FLY_MAX_TPS = 2800;        // 6000 rpm
    static final double NECTAR_TEST_TPS = 3800 * TICKS_PER_REV / 60;   // test speed NECTAR 3800 rpm (set 11-12)
    static final double POLLEN_TEST_TPS = 5500 * TICKS_PER_REV / 60;   // test speed POLLEN 5500 rpm (set 11-12)
    static final double NECTAR_MATCH_TPS = 0.80 * FLY_MAX_TPS;   // match speed about 4800 rpm, set after shooting tests
    static final double POLLEN_MATCH_TPS = 0.80 * FLY_MAX_TPS;
    static final double FLY_STEP_TPS = 100.0 * TICKS_PER_REV / 60;   // 100 rpm per press
    static final double FLY_RAMP_TPS_PER_SEC = FLY_MAX_TPS;          // 0 -> full speed takes at least 1 second
    static final double AT_SPEED_BAND = 0.03;
    static final double GUARD_MIN_TPS = 300, GUARD_RATIO = 0.25, GUARD_SECONDS = 1.5;
    static final boolean NECTAR_REVERSED = true;    // 11-11 tested on robot: was backwards
    static final boolean POLLEN_REVERSED = false;
    static final double FLY_P = 10, FLY_I = 1, FLY_D = 0, FLY_F = 32767.0 / FLY_MAX_TPS;   // not tuned yet
```
- **Line 114** — 28 encoder ticks per shaft revolution (the rpm↔tps conversion factor).
- **Line 115** — Top speed ≈ 2800 tps = 6000 rpm; used for the F constant and clamps.
- **Line 116** — NECTAR test speed: 3800 rpm converted to tps (the arithmetic runs once at compile/startup).
- **Line 117** — POLLEN test speed: 5500 rpm → tps.
- **Lines 118–119** — "Match" speeds (80 % of max) — see the dead-code note in the Evaluation; currently unused at runtime because the Y toggle was removed.
- **Line 120** — Each dpad press changes a target by 100 rpm (in tps).
- **Line 121** — Ramp rate: command moves at most 2800 tps per second ⇒ 0→full takes ≥1 s — the coupler protection.
- **Line 122** — "At speed" = ±3 %.
- **Line 123** — Stall guard: only applies when commanded >300 tps; "stalled" = actual <25 % of commanded; must persist 1.5 s.
- **Line 124** — NECTAR motor needed reversal (found 11-11).
- **Line 125** — POLLEN didn't.
- **Line 126** — Flywheel PIDF: F = 32767 / max-tps (feed-forward that alone nearly holds speed); P/I correct; "not tuned yet".

### Intake (128–130)

```java
    static final double INTAKE_POWER = 1.0;
    static final double INTAKE_SPIT_POWER = -1.0;   // 11-12: reverse at full power
    static final boolean INTAKE_REVERSED = true;      // 11-11 tested on robot: was backwards
```
- **Line 128** — Full power forward (swallowing).
- **Line 129** — Full power reversed (used by the B unjam backup).
- **Line 130** — Intake needed reversal (found 11-11).

### Servo defaults (132–137)

```java
    // Servo open / closed: first use the numbers Car1 Hardware Test saved on the Hub in /sdcard/FIRST/car1_servo.txt;
    // only if nothing is saved, use the values below
    static final double GATE_L_OPEN = 0.90, GATE_L_CLOSED = 0.78; // 11-11 tested on robot
    static final double GATE_R_OPEN = 0.46, GATE_R_CLOSED = 0.56; // 11-11 tested on robot
    static final double END_L_OPEN = 0.3, END_L_CLOSED = 0.7;     // placeholder
    static final double END_R_OPEN = 0.3, END_R_CLOSED = 0.7;     // placeholder
```
- **Lines 132–133** — The rule: saved file wins, constants are the fallback.
- **Line 134** — Left gate (NECTAR) — tuned 11-11.
- **Line 135** — Right gate (POLLEN) — tuned 11-11. Note open < closed here while the left gate is the opposite — servos differ in mounting; that's exactly why the values live in a file.
- **Lines 136–137** — End-servo placeholders (the hardware isn't fitted on Car 1 yet).

### v3 shooting/logging constants (139–145)

```java
    // v3: one-button shooting / shot log (not tested yet)
    static final double FEED_MS = 250;        // how long the gates open each time in one-button shooting; test: shortest time that lets out only 1 ball
    static final double FEED_GAP_MS = 150;    // minimum wait after closing before opening again (plus waiting to get back to speed)
    static final double SHOT_DIP = 0.05;      // after at speed, a drop of more than 5% counts as a shot
    static final double LOW_VOLT = 12.0;
    static final String FLY_FILE = "/sdcard/FIRST/car1_fly.txt";
    static final String SHOT_FILE = "/sdcard/FIRST/car1_shots.csv";
```
- **Line 140** — Burst feed: gates open 250 ms per ball (a tuning target: shortest time that drops exactly one ball).
- **Line 141** — Minimum closed time between feeds.
- **Line 142** — Shot detection threshold: a >5 % speed dip counts as one shot.
- **Line 143** — Low-battery warning level.
- **Line 144** — The (currently disabled) flywheel-speed save file.
- **Line 145** — The shot log — note `.csv`: commas, so it opens straight into a spreadsheet.

## Lines 148–174 — fields

```java
    private DcMotorEx lf, lb, rf, rb, flyN, flyP, intake;
    private Servo gateL, gateR, endL, endR;
```
- **Line 148** — All 7 motors.
- **Line 149** — All 4 servos.

```java
    private final boolean[] servoArmed = new boolean[4];   // no servo signal until the first button press
```
- **Line 150** — Per-servo "has been commanded yet" flag — the don't-move-at-startup safety.

```java
    private IMU imu;
    private GoBildaPinpointDriver pp;
```
- **Lines 151–152** — Heading source and odometry (display + optional heading).

```java
    private boolean tailFirst = false;   // back of robot is the front
    private boolean fieldCentric = false, intakeOn = false, flyOn = false, matchSpeed = false, rumbled = false;
```
- **Line 154** — Back-is-front mode (default off).
- **Line 155** — The mode toggles and states: field-centric off by default, intake off, flywheels off, `matchSpeed` false (permanently, since Y was removed), `rumbled` = already buzzed for this at-speed event.

```java
    private double nectarTest = NECTAR_TEST_TPS, pollenTest = POLLEN_TEST_TPS, nectarMatch = NECTAR_MATCH_TPS, pollenMatch = POLLEN_MATCH_TPS;
    private double cmdN = 0, cmdP = 0;   // current command after the slow ramp
```
- **Line 156** — The *working copies* of the speed targets (dpad trims change these, not the constants).
- **Line 157** — What the ramp has actually commanded so far — starts at 0 and climbs.

```java
    private String flyAlarm = null;
```
- **Line 158** — Non-null = the stall guard fired; the text is shown on screen until X restarts.

```java
    private final ElapsedTime guardN = new ElapsedTime(), guardP = new ElapsedTime(), loop = new ElapsedTime();
```
- **Line 159** — Three stopwatches: how long each flywheel has been "stalled", and the loop-time timer (for dt).

```java
    private final Gamepad prev1 = new Gamepad(), prev2 = new Gamepad(), cur1 = new Gamepad(), cur2 = new Gamepad();
```
- **Line 161** — Four whole-gamepad snapshots: current and previous for both pads. Copying the entire gamepad object once per cycle and comparing fields is the FTC-recommended edge-detection pattern — no need for 20 separate pButton booleans.

### v3 fields (163–174)

```java
    // v3
    static final double GATE_WAIT_S = 0.5;   // bumpers / burst: if the flywheel is not at speed, wait at most this long, then open anyway (11-12)
    private double holdLT = 0, holdRT = 0;
    private boolean holdLWas = false, holdRWas = false;
    private boolean latchL = false, latchR = false;       // bumpers: once opened at speed, stay open until released
    private boolean burstOpen = false;                    // one-button shooting: gates are open right now
    private final ElapsedTime burstT = new ElapsedTime(), matchT = new ElapsedTime();
    private boolean warn30 = false, warn10 = false;
    private VoltageSensor[] volts = new VoltageSensor[0];
    private final ShotTrack shotN = new ShotTrack("NECTAR"), shotP = new ShotTrack("POLLEN");
    private final List<String> shotLog = new ArrayList<>();
    private String flyFileMsg = "";
```
- **Line 164** — Max time a held bumper waits for flywheel speed before opening anyway (added after the 11-12 discovery that POLLEN at 5500 rpm never fit inside ±3 %).
- **Line 165** — When each bumper press started (timestamps).
- **Line 166** — Previous-cycle bumper states (edge detection).
- **Line 167** — The latch: once a gate opened (at speed), it stays open until release — so a mid-shot speed wobble can't pinch a ball.
- **Line 168** — Whether the burst feeder currently has the gates open.
- **Line 169** — `burstT` times both the open pulse and the closed gap (reset at each transition); `matchT` times the match for the 1:30/1:50 warnings.
- **Line 170** — One-shot flags for the two time warnings.
- **Line 171** — All voltage sensors found (both hubs report).
- **Line 172** — One shot-tracker per flywheel.
- **Line 173** — Shot CSV lines collected in memory until the program ends.
- **Line 174** — Message about the (disabled) flywheel-speed file.

## Lines 176–207 — ShotTrack: the shot detector

```java
    /** Shot log for one flywheel: after at speed, a drop of more than SHOT_DIP starts a shot; back at speed ends it */
    private final class ShotTrack {
```
- **Line 176** — Javadoc: the detection rule.
- **Line 177** — An *inner* class (no `static` — it reads the outer object's `flyOn`, `voltage()`, etc.). One instance per flywheel.

```java
        final String name;
        boolean armed = false, inShot = false;
        double minTps = 0, target = 0;
        final ElapsedTime t = new ElapsedTime();
        int count = 0;
        String last = "none yet";
```
- **Line 178** — Which wheel ("NECTAR"/"POLLEN") — used in logs.
- **Line 179** — `armed` = "we've seen it at speed, dips now count"; `inShot` = "a dip is in progress".
- **Line 180** — Lowest speed seen during the shot; the target this cycle.
- **Line 181** — Times the shot (from dip to recovery).
- **Line 182** — Lifetime shot count.
- **Line 183** — The last shot's one-line summary, for the screen.

```java
        ShotTrack(String name) { this.name = name; }
```
- **Line 184** — Constructor: just stores the name.

```java
        void reset() { armed = false; inShot = false; }
```
- **Line 186** — Forget progress (wheel off / target changed) without wiping the count.

```java
        void update(DcMotorEx m, double tgt) {
            if (m == null || !flyOn || tgt <= 0) { reset(); return; }
            if (Math.abs(tgt - target) > 1) { target = tgt; reset(); }   // target changed: start again
```
- **Line 188** — Called once per loop for each wheel. Motor missing / flywheels off / no target → not watching.
- **Line 190** — A dpad trim changed the target (>1 tps difference)? The "at speed" reference moved — re-arm from scratch so the trims themselves don't register as dips.

```java
            double v = m.getVelocity();
            if (!inShot) {
                if (atSpeed(m, tgt)) armed = true;
                else if (armed && v < tgt * (1 - SHOT_DIP)) { inShot = true; minTps = v; t.reset(); }
            } else {
```
- **Line 191** — Read the actual speed.
- **Line 192** — Not currently in a shot: if at speed → we're armed (a following dip is meaningful).
- **Line 194** — Armed and speed fell >5 % below target → **a shot starts** (a ball absorbed energy): record the entry speed, start the clock.

```java
                minTps = Math.min(minTps, v);
                if (atSpeed(m, tgt)) {
                    inShot = false; count++;
                    double tr = t.milliseconds();
                    last = String.format(Locale.US, "#%d lowest %.0f rpm (drop %.0f%%) recover %.0f ms", count,
                            minTps * 60 / TICKS_PER_REV, 100 * (1 - minTps / tgt), tr);
                    shotLog.add(String.format(Locale.US, "%s,%d,%.0f,%.0f,%.1f,%.0f,%.2f,%.1f", name, count,
                            tgt * 60 / TICKS_PER_REV, minTps * 60 / TICKS_PER_REV, 100 * (1 - minTps / tgt), tr, voltage(), matchT.seconds()));
                } else if (t.seconds() > 3) { inShot = false; armed = false; }   // not back after 3 seconds: don't count it (maybe jammed or power cut)
            }
        }
```
- **Line 196** — Track the lowest speed of the shot.
- **Line 197** — Back at speed → the ball has left: **the shot ends**.
- **Line 198** — Count it.
- **Line 199** — Recovery time in ms.
- **Lines 200–201** — Human-readable summary for the screen (converted to rpm).
- **Lines 202–203** — The CSV row: wheel, shot#, target rpm, lowest rpm, drop %, recovery ms, battery voltage, match time — everything needed to tune gates and speeds later.
- **Line 204** — No recovery after 3 s → not a real shot (jam or power cut); forget the whole event and require a fresh at-speed.

## Lines 209–272 — runOpMode(): setup and the main loop

```java
    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
```
- **Line 211** — Bulk caching on all hubs — faster loop.

```java
        lf = motor("lf", LEFT_REVERSED, true);
        lb = motor("lb", LEFT_REVERSED, true);
        rf = motor("rf", RIGHT_REVERSED, true);
        rb = motor("rb", RIGHT_REVERSED, true);
        flyN = flywheel("flywheelNectar", NECTAR_REVERSED);
        flyP = flywheel("flywheelPollen", POLLEN_REVERSED);
        intake = motor("intake", INTAKE_REVERSED, false);
        gateL = servo("sorterGateL");
        gateR = servo("sorterGateR");
        endL = servo("sorterEndL");
        endR = servo("sorterEndR");
```
- **Lines 213–216** — Four drive motors via `motor(name, reversed, brake=true)`: BRAKE stop so the robot halts crisply.
- **Lines 217–218** — Two flywheels via the special `flywheel()` helper (encoder mode + PIDF — line 536).
- **Line 219** — Intake with `brake=false` → FLOAT (coasts; gentler on a roller).
- **Lines 220–223** — Four servos via the simple `servo()` helper — grabbed but **not commanded** (the arming safety).

```java
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
            imu.resetYaw();
        } catch (Exception e) { imu = null; }
```
- **Line 226** — Get the built-in IMU by name.
- **Line 227** — Initialise it with the hub's physical orientation (the two "not measured" constants) — the IMU needs this to know which axis is yaw on *this* robot.
- **Line 228** — Zero the yaw.
- **Line 229** — Any failure → `null` (field-centric just won't work; robot-centric drive is unaffected).

```java
        try {
            pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
            pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            pp.setOffsets(15, 55, DistanceUnit.MM);   // numbers from the old chassis, new chassis not measured (only affects position, not heading)
            pp.resetPosAndIMU();
        } catch (Exception e) { pp = null; }
```
- **Lines 231–236** — Pinpoint setup: pod type, both pods reversed (the PP Test result), mounting offsets (old chassis numbers — with the honest note that they only affect position-during-turns, not heading), then zero. Failure → `null` (only the screen readout is lost).

```java
        volts = hardwareMap.getAll(VoltageSensor.class).toArray(new VoltageSensor[0]);
```
- **Line 238** — Collect every voltage sensor (each hub has one).

```java
        // 11-12: "always start with these speeds, nothing else" -> saved speeds are not loaded; every start is NECTAR 3800 / POLLEN 5500
        // loadFly();
        ServoStore.load();
```
- **Lines 239–240** — The saved-speeds feature, deliberately switched off: a team decision ("always start with these speeds") — the call is left in place, commented.
- **Line 241** — DO load the servo numbers — those are still wanted.

```java
        while (opModeInInit()) {
            telemetry.addLine("Car1 TeleOp v3 ready, press PLAY");
            telemetry.addLine("Mechanism buttons work on both gamepads: A intake | B back up + push | X flywheels | bumpers feed | right trigger burst shooting");
            telemetry.addData("Flywheel speed", "NECTAR %.0f | POLLEN %.0f rpm", rpm(nectarTest), rpm(pollenTest));
            telemetry.addData("Battery", "%.2f V", voltage());
            telemetry.addData("Found", found());
            telemetry.addData("Gate values", "left open %s closed %s | right open %s closed %s (not saved = placeholder)",
                    ServoStore.fmt("sorterGateL", true), ServoStore.fmt("sorterGateL", false),
                    ServoStore.fmt("sorterGateR", true), ServoStore.fmt("sorterGateR", false));
            telemetry.update();
            sleep(50);
        }
```
- **Line 242** — The INIT screen loop (~20 Hz).
- **Line 243** — Version banner.
- **Line 244** — Button cheat-sheet.
- **Line 245** — The starting flywheel speeds (rpm() converts back for display).
- **Line 246** — Battery right away — catch a bad battery before the match.
- **Line 247** — `found()` — the any-devices-missing report.
- **Lines 248–250** — Which gate numbers will be used (from the saved file, or "not saved").

```java
        guardN.reset(); guardP.reset(); loop.reset(); matchT.reset();
```
- **Line 254** — Zero all the stopwatches at PLAY: stall guards start fresh, loop timer starts, match clock starts (the 1:30/1:50 warnings are measured from here).

```java
        while (opModeIsActive()) {
            cur1.copy(gamepad1); cur2.copy(gamepad2);
            if (pp != null) pp.update();
            double dt = Math.min(loop.seconds(), 0.1); loop.reset();
            drive();
            intakeControl();
            flywheelControl(dt);
            sorterControl();
            shotN.update(flyN, targetN());
            shotP.update(flyP, targetP());
            timeWarn();
            show();
            prev1.copy(cur1); prev2.copy(cur2);
        }
        stopAll();
        saveShots();
```
- **Line 256** — The main loop — everything TeleOp does happens in here, ~50×/s.
- **Line 257** — Snapshot both gamepads — this cycle's "current".
- **Line 258** — Poll the Pinpoint.
- **Line 259** — Loop time `dt`, capped at 0.1 s — so after a garbage-collection pause the flywheel ramp takes one bounded step, not a jump.
- **Line 260** — Drive (reads sticks, writes wheels).
- **Line 261** — Intake + unjam state machine.
- **Line 262** — Flywheels (targets, ramp, guard, buzz).
- **Line 263** — Gates + burst feeding.
- **Lines 264–265** — Shot detection for each wheel, using the current targets.
- **Line 266** — Time rumble checks.
- **Line 267** — Draw the screen.
- **Line 268** — This cycle's snapshots become next cycle's "previous" — closing the edge-detection loop.
- **Line 270** — STOP → all motor power 0.
- **Line 271** — Write the shot CSV to the Hub.

## Lines 275–279 — timeWarn(): the end-game rumbles

```java
    private void timeWarn() {
        double s = matchT.seconds();
        if (!warn30 && s >= 90) { warn30 = true; gamepad1.rumbleBlips(3); gamepad2.rumbleBlips(3); }
        if (!warn10 && s >= 110) { warn10 = true; gamepad1.rumbleBlips(6); gamepad2.rumbleBlips(6); }
    }
```
- **Line 276** — Seconds since PLAY.
- **Line 277** — At 1:30 (endgame begins in a 2:00 match): 3 rumble blips, once (the flag makes it one-shot).
- **Line 278** — At 1:50: 6 faster blips. You feel the countdown without looking.

## Lines 281–287 — voltage() and rpm()

```java
    private double voltage() {
        double v = 0;
        for (VoltageSensor s : volts) { double x = s.getVoltage(); if (x > 0 && (v == 0 || x < v)) v = x; }
        return v;
    }
```
- **Line 283** — Of all sensors reporting a sensible (>0) voltage, return the **lowest** — the battery is only as good as the worst reading (and a half-second dropout on one hub shows up here first).

```java
    private static double rpm(double tps) { return tps * 60 / TICKS_PER_REV; }
```
- **Line 287** — tps → rpm for display.

## Lines 289–305 — the (disabled) flywheel-speed file functions

```java
    private void loadFly() {
        File f = new File(FLY_FILE);
        if (!f.exists()) { flyFileMsg = "(nothing saved, using program defaults)"; return; }
        try (BufferedReader r = new BufferedReader(new FileReader(f))) {
            String[] p = r.readLine().trim().split(" +");
            nectarTest = Double.parseDouble(p[0]); pollenTest = Double.parseDouble(p[1]);
            nectarMatch = Double.parseDouble(p[2]); pollenMatch = Double.parseDouble(p[3]);
            flyFileMsg = "(saved on Hub)";
        } catch (Exception e) { flyFileMsg = "(error reading saved file, using defaults)"; }
    }
```
- **Lines 289–298** — Read four speeds from one line of `car1_fly.txt`. **Currently dead code** — the only call (line 240) is commented out per the 11-12 team decision.

```java
    private void saveFly() {
        try (FileWriter w = new FileWriter(FLY_FILE, false)) {
            w.write(String.format(Locale.US, "%.1f %.1f %.1f %.1f%n", nectarTest, pollenTest, nectarMatch, pollenMatch));
            flyFileMsg = "saved";
        } catch (Exception e) { flyFileMsg = "save failed " + e.getMessage(); }
    }
```
- **Lines 300–305** — Write those four numbers back. Also currently uncalled — kept for when the feature returns.

## Lines 307–316 — saveShots(): the CSV

```java
    private void saveShots() {
        if (shotLog.isEmpty()) return;
        File f = new File(SHOT_FILE);
        boolean head = !f.exists();
        String stamp = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date());
        try (FileWriter w = new FileWriter(f, true)) {
            if (head) w.write("time,wheel,n,target_rpm,min_rpm,drop_pct,recover_ms,volt,t_s\n");
            for (String s : shotLog) w.write(stamp + "," + s + "\n");
        } catch (Exception ignored) { }
    }
```
- **Line 308** — No shots → no file.
- **Line 310** — Header needed only when the file is new (so it's written once, not per session).
- **Line 311** — One timestamp for the whole session.
- **Line 313** — The CSV column names.
- **Line 314** — Every shot line, prefixed with the timestamp. Append mode keeps sessions separated by their timestamps. Failure ignored — logging must not crash the robot.

## Line 319 — pressed(): the either-gamepad edge detector

```java
    private boolean pressed(boolean c1, boolean p1, boolean c2, boolean p2) { return (c1 && !p1) || (c2 && !p2); }
```
- **Line 319** — "Was this button just pressed, on EITHER pad?" — rising edge on pad1 or rising edge on pad2. This one line is what makes every mechanism button work on both gamepads.

## Lines 322–348 — drive(): the driving math

```java
    private void drive() {
        if (cur1.right_stick_button && !prev1.right_stick_button) fieldCentric = !fieldCentric;
        if (cur1.back && !prev1.back) resetHeading();
```
- **Line 323** — Right-stick press toggles field-centric.
- **Line 324** — Back press re-zeros the heading (used when the robot is placed facing a known direction).

```java
        // 11-12: press the left stick once -> back of robot is the front (stick forward = drive toward the back,
        // left / right also flip); press again to go back to normal. Makes it easy to aim the back at the goal.
        if (cur1.left_stick_button && !prev1.left_stick_button) {
            tailFirst = !tailFirst;
            if (tailFirst) gamepad1.rumbleBlips(2); else gamepad1.rumble(300);
        }
```
- **Lines 327–330** — Left-stick press toggles back-is-front; the two feedback rumbles (2 blips on, one 300 ms buzz off) tell you the state without looking down.

```java
        double y = -cur1.left_stick_y;
        double x = cur1.left_stick_x * STRAFE_FIX;
        double r = cur1.right_stick_x;
```
- **Line 332** — Forward axis: stick-up reads negative, so negate → push up = +forward.
- **Line 333** — Strafe axis, right = +, boosted 10 % (mecanum strafe weakness).
- **Line 334** — Turn axis: right stick left/right, right = + (clockwise).

```java
        if (tailFirst && !fieldCentric) { y = -y; x = -x; }   // turning does not flip: seen from above, clockwise is still clockwise
```
- **Line 335** — Back-is-front: flip forward/back and left/right — but NOT turn (from above, clockwise is clockwise whichever way the chassis points).

```java
        if (fieldCentric) {
            double h = heading();
            double rx = x * Math.cos(-h) - y * Math.sin(-h);
            double ry = x * Math.sin(-h) + y * Math.cos(-h);
            x = rx; y = ry;
        }
```
- **Lines 336–341** — Field-centric: get the robot's heading, then rotate the stick vector by −h so "stick up" always means "away from the driver no matter how the robot is turned". The two middle lines are a standard 2-D rotation.

```java
        double scale = cur1.left_trigger > 0.5 ? SLOW_SCALE : 1.0;
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        setPower(lf, (y + x + r) / d * scale);
        setPower(lb, (y - x + r) / d * scale);
        setPower(rf, (y - x - r) / d * scale);
        setPower(rb, (y + x - r) / d * scale);
    }
```
- **Line 342** — Half-pull the left trigger → 40 % power.
- **Line 343** — Normaliser: the sum of magnitudes, min 1 — bigger-than-full combined commands shrink proportionally; small ones pass through.
- **Lines 344–347** — The mecanum mix (identical to the other two programs) × slow scale.

## Lines 350–359 — heading() and resetHeading()

```java
    private double heading() {
        if (USE_PP_HEADING && pp != null) return pp.getPosition().getHeading(AngleUnit.RADIANS);
        if (imu != null) return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return 0;
    }
```
- **Line 351** — If configured for Pinpoint heading and it exists → use it.
- **Line 352** — Otherwise the IMU's yaw in radians.
- **Line 353** — Neither → 0 (field-centric silently becomes robot-centric).

```java
    private void resetHeading() {
        if (imu != null) imu.resetYaw();
        if (pp != null) pp.resetPosAndIMU();
    }
```
- **Lines 356–359** — Zero whichever heading sources exist — both, so they stay consistent.

## Lines 361–383 — intakeControl() and the unjam state machine

```java
    // ---------------- intake ----------------
    // 11-12: when a ball is stuck between the intake and the sorter, back up a little then push it in, without spitting the ball out.
    // Press B: reverse UNJAM_REV_S -> forward UNJAM_FWD_S -> back to the old on/off ("hold to spit" was removed).
    static final double UNJAM_TAP_S = 0.4, UNJAM_REV_S = 0.25, UNJAM_FWD_S = 1.0;
    private int unjam = 0;            // 0 none, 1 backing up, 2 pushing
    private double bDownT = 0, unjamEnd = 0;
    private boolean bWas = false;
```
- **Lines 362–363** — The unjam idea and its history (the old "hold B to spit" was removed 11-12).
- **Line 364** — The three timings: (UNJAM_TAP_S is declared but unused — leftover from the removed hold-to-spit feature); reverse for 0.25 s, then push for 1.0 s.
- **Line 365** — The state variable: 0 = normal, 1 = backing up, 2 = pushing.
- **Line 366** — Timestamps: when B went down; when the current unjam phase ends.
- **Line 367** — Previous-cycle B state.

```java
    private void intakeControl() {
        double now = getRuntime();
        if (pressed(cur1.a, prev1.a, cur2.a, prev2.a)) { intakeOn = !intakeOn; unjam = 0; }
        if (cur2.back && !prev2.back) { intakeOn = false; flyOn = false; unjam = 0; }
        boolean b = cur1.b || cur2.b;
        // 11-12: "remove the spitting for now" -> B no longer spits while held; a press is always back up + push (holding time does not matter)
        if (b && !bWas) { bDownT = now; unjam = 1; unjamEnd = now + UNJAM_REV_S; }
        bWas = b;
        if (unjam == 1 && now >= unjamEnd) { unjam = 2; unjamEnd = now + UNJAM_FWD_S; }
        if (unjam == 2 && now >= unjamEnd) unjam = 0;

        if (unjam == 1) setPower(intake, INTAKE_SPIT_POWER);
        else if (unjam == 2) setPower(intake, INTAKE_POWER);
        else setPower(intake, intakeOn ? INTAKE_POWER : 0);
    }
```
- **Line 370** — `getRuntime()` = seconds since the op mode started — a free clock for the state machine.
- **Line 371** — A press (either pad): toggle intake; a toggle also cancels any unjam in progress.
- **Line 372** — Operator's back = the panic button: intake off, flywheels off, unjam cancelled.
- **Line 373** — B is "held on either pad".
- **Line 375** — B **press** → enter state 1 (backing up); note the phase end time = now + 0.25 s.
- **Line 376** — Remember B for next cycle's edge.
- **Line 377** — State 1 time expired → state 2 (pushing), end = now + 1.0 s.
- **Line 378** — State 2 expired → state 0.
- **Lines 380–382** — Apply: state 1 → full reverse; state 2 → full forward; state 0 → whatever the toggle says. A two-phase sequence running from a single button press — the whole "state machine" idea in 5 lines.

## Lines 385–422 — flywheelControl(): targets, ramp, guard, buzz

```java
    private void flywheelControl(double dt) {
        if (pressed(cur1.x, prev1.x, cur2.x, prev2.x)) { flyOn = !flyOn; flyAlarm = null; rumbled = false; guardN.reset(); guardP.reset(); }
```
- **Line 387** — X press (either pad): toggle flywheels; turning on (or off) also clears any stall alarm, re-arms the at-speed buzz, and restarts the stall timers.

```java
        // One-button shooting: the moment the right trigger is pressed, turn on the flywheels if they are off
        if (pressed(cur1.right_trigger > 0.5, prev1.right_trigger > 0.5, cur2.right_trigger > 0.5, prev2.right_trigger > 0.5) && !flyOn) {
            flyOn = true; flyAlarm = null; rumbled = false; guardN.reset(); guardP.reset();
        }
```
- **Line 389** — Edge-detect the trigger crossing half-pull (booleans fed into the same `pressed` helper — it works for any boolean condition).
- **Lines 389–391** — Trigger press with flywheels off → turn them on (same cleanup as X) — so the burst button alone is enough to start shooting from a dead stop.

```java
        // Y speed switch was removed (11-12: only NECTAR 3800 / POLLEN 5500)
        double sN = 0, sP = 0;
        if (pressed(cur1.dpad_up, prev1.dpad_up, cur2.dpad_up, prev2.dpad_up)) sN += FLY_STEP_TPS;
        if (pressed(cur1.dpad_down, prev1.dpad_down, cur2.dpad_down, prev2.dpad_down)) sN -= FLY_STEP_TPS;
        if (pressed(cur1.dpad_right, prev1.dpad_right, cur2.dpad_right, prev2.dpad_right)) sP += FLY_STEP_TPS;
        if (pressed(cur1.dpad_left, prev1.dpad_left, cur2.dpad_left, prev2.dpad_left)) sP -= FLY_STEP_TPS;
        if (matchSpeed) { nectarMatch = clip(nectarMatch + sN, 0, FLY_MAX_TPS); pollenMatch = clip(pollenMatch + sP, 0, FLY_MAX_TPS); }
        else { nectarTest = clip(nectarTest + sN, 0, FLY_MAX_TPS); pollenTest = clip(pollenTest + sP, 0, FLY_MAX_TPS); }
        // Speeds changed with the dpad only last for this run and are not saved (11-12)
```
- **Line 392** — History note: the Y toggle is gone; there is one speed per wheel per run.
- **Line 393** — This cycle's step deltas (0 if no dpad press).
- **Lines 394–397** — Each dpad direction nudges its wheel's target by ±100 rpm (either pad).
- **Line 398** — The `matchSpeed` branch — permanently false now (dead but harmless).
- **Line 399** — The live branch: trim the test targets, clamped to 0…max.
- **Line 400** — Trims are per-run only — a restart returns to 3800/5500.

```java
        double step = FLY_RAMP_TPS_PER_SEC * dt;
        cmdN = flyOn ? ramp(cmdN, targetN(), step) : 0;
        cmdP = flyOn ? ramp(cmdP, targetP(), step) : 0;
        spin(flyN, cmdN);
        spin(flyP, cmdP);
```
- **Line 402** — This cycle's allowed ramp step = rate × loop time (e.g. 2800 × 0.02 = 56 tps per cycle).
- **Line 403** — NECTAR's commanded speed climbs toward its target by at most `step`; off → command 0 immediately.
- **Line 404** — Same for POLLEN.
- **Lines 405–406** — Push the commands to the motors.

```java
        if (flyOn) {
            if (!stalled(flyN, cmdN)) guardN.reset();
            if (!stalled(flyP, cmdP)) guardP.reset();
            if (guardN.seconds() > GUARD_SECONDS || guardP.seconds() > GUARD_SECONDS) {
                flyAlarm = (guardN.seconds() > GUARD_SECONDS ? "NECTAR" : "POLLEN") + " flywheel won't spin up (encoder not plugged in / wrong direction / jammed), power cut, press X to restart";
                flyOn = false; cmdN = 0; cmdP = 0;
                spin(flyN, 0); spin(flyP, 0);
                gamepad1.rumble(600); gamepad2.rumble(600);
            }
        }
```
- **Line 408** — Guard runs only while on.
- **Lines 409–410** — Each cycle a wheel is NOT stalled, its timer resets — so the alarm needs a *continuous* 1.5 s stall (spin-up lag never lasts that long).
- **Line 411** — Either wheel stalled 1.5 s…
- **Line 412** — …write a loud alarm naming the wheel and the likely causes.
- **Line 413** — Cut everything (flyOn=false, commands 0).
- **Line 414** — Actually zero the motors.
- **Line 415** — Long rumble on both pads — you feel the failure.

```java
        // Buzz only the first time it reaches speed after "flywheels on / speed change"; small ups and downs
        // around the target after that do not buzz again (11-12 feedback)
        if (sN != 0 || sP != 0) rumbled = false;
        if (flyReady() && !rumbled) { gamepad1.rumble(250); gamepad2.rumble(250); rumbled = true; }
    }
```
- **Lines 418–419** — Any dpad trim re-arms the buzz (a new target deserves a new "ready" signal).
- **Line 421** — Both wheels at speed + not yet buzzed → short rumble on both pads, once. The drivers *feel* "ready to feed" without watching the screen.

## Lines 424–443 — the small flywheel helpers

```java
    private double targetN() { return matchSpeed ? nectarMatch : nectarTest; }
    private double targetP() { return matchSpeed ? pollenMatch : pollenTest; }
```
- **Lines 424–425** — Which target is live (given matchSpeed is always false: the test speeds).

```java
    private boolean flyReady() {
        return flyOn && atSpeed(flyN, targetN()) && atSpeed(flyP, targetP());
    }
```
- **Lines 427–429** — "Both wheels at speed" — gates and the buzz key off this.

```java
    private void spin(DcMotorEx m, double tps) {
        if (m == null) return;
        if (flyOn && tps > 0) m.setVelocity(tps);
        else m.setPower(0);   // setPower(0) + FLOAT = coast to stop; setVelocity(0) would brake
    }
```
- **Lines 431–435** — Null-safe motor command — with the comment explaining a subtle but important choice: `setVelocity(0)` would command "hold 0 speed" = active braking; `setPower(0)` with FLOAT coasts. Flywheels must coast.

```java
    private static boolean stalled(DcMotorEx m, double tps) {
        return m != null && tps > GUARD_MIN_TPS && m.getVelocity() < tps * GUARD_RATIO;
    }
```
- **Lines 437–439** — The stall predicate: motor present, commanded meaningfully (>300 tps), actual <25 % of commanded.

```java
    private static boolean atSpeed(DcMotorEx m, double tps) {
        return m == null || Math.abs(m.getVelocity() - tps) <= AT_SPEED_BAND * tps;
    }
```
- **Lines 441–443** — Within ±3 % of target; a missing motor counts as at-speed (never block the human on missing hardware).

## Lines 446–470 — sorterControl(): gates and burst feeding

```java
    private void sorterControl() {
        // Bumpers: flywheels off -> opens while held (to clear balls); flywheels on -> opens only when that wheel is
        // at speed, then stays open until released (so it does not pinch a ball that is passing through)
        boolean holdL = cur1.left_bumper || cur2.left_bumper, holdR = cur1.right_bumper || cur2.right_bumper;
```
- **Lines 447–448** — The two behaviours, in words.
- **Line 449** — Is each bumper held (either pad)?

```java
        // 11-12 tested on robot: POLLEN at 5500 rpm never got inside +/-3%, so the gate never opened. Changed to:
        // open when at speed, OR open anyway after holding longer than GATE_WAIT_S.
        double now = getRuntime();
        if (holdL && !holdLWas) holdLT = now;
        if (holdR && !holdRWas) holdRT = now;
        holdLWas = holdL; holdRWas = holdR;
```
- **Lines 450–451** — The real-world fix: at 5500 rpm the flywheel rarely settles inside ±3 %, so the strict rule starved the gate — now there's a 0.5 s "open anyway" fallback.
- **Line 452** — The clock.
- **Line 453** — Left bumper **press** → remember when the hold started.
- **Line 454** — Same for right.
- **Line 455** — Update hold-state memory.

```java
        latchL = holdL && (latchL || !flyOn || atSpeed(flyN, targetN()) || now - holdLT >= GATE_WAIT_S);
        latchR = holdR && (atSpeed... same shape for the right side);
```
- **Line 456** — The latch (left): the gate is open while held AND any of: it was already open (latch), flywheels are off (clearing mode), that wheel is at speed, or you've held 0.5 s anyway. Once any condition opens it, the first term keeps it open for the rest of the hold — speed wobble can't close it on a ball mid-feed.
- **Line 457** — The same for the right gate with the POLLEN wheel.

```java
        // One-button shooting: hold the right trigger -> both at speed and at least FEED_GAP_MS since the last close
        // -> both gates open for FEED_MS -> close -> repeat
        boolean burst = cur1.right_trigger > 0.5 || cur2.right_trigger > 0.5;
        if (!burst || !flyOn) burstOpen = false;
        else if (burstOpen) { if (burstT.milliseconds() >= FEED_MS) { burstOpen = false; burstT.reset(); } }
        else if ((flyReady() && burstT.milliseconds() >= FEED_GAP_MS) || burstT.milliseconds() >= FEED_GAP_MS + GATE_WAIT_S * 1000) { burstOpen = true; burstT.reset(); }   // if not at speed, wait at most GATE_WAIT_S extra
```
- **Lines 459–460** — The burst cycle, in words.
- **Line 461** — Trigger held (either pad, >half pull)?
- **Line 462** — Released or flywheels off → gates closed, machine idle.
- **Line 463** — Currently open → stay open until FEED_MS (250 ms) elapsed, then close and restart the timer (it now times the gap).
- **Line 464** — Currently closed → open when (at speed AND gap ≥150 ms) — or, if speed never comes, after 150 ms + 500 ms anyway. Then restart the timer (it now times the pulse). Holding the trigger = open-close-open-close… one ball per cycle.

```java
        servoCtl(0, gateL, latchL || burstOpen, ServoStore.get("sorterGateL", true, GATE_L_OPEN), ServoStore.get("sorterGateL", false, GATE_L_CLOSED));
        servoCtl(1, gateR, latchR || burstOpen, ServoStore.get("sorterGateR", true, GATE_R_OPEN), ServoStore.get("sorterGateR", false, GATE_R_CLOSED));
        servoCtl(2, endL, cur2.left_trigger > 0.5, ServoStore.get("sorterEndL", true, END_L_OPEN), ServoStore.get("sorterEndL", false, END_L_CLOSED));
        servoCtl(3, endR, cur2.right... left_trigger > 0.5, ServoStore.get("sorterEndR", true, END_R_OPEN), ServoStore.get("sorterEndR", false, END_R_CLOSED));
```
- **Line 466** — Left gate: open if (bumper latch) OR (burst cycle has it open); positions from the saved file or defaults.
- **Line 467** — Right gate: same.
- **Line 468** — End servos: open while the **operator's** left trigger is held (gamepad2 only) — hardware not fitted yet, so this is dormant.
- **Line 469** — Same for the right end servo.

## Lines 472–476 — servoCtl(): the arming gate

```java
    private void servoCtl(int i, Servo s, boolean open, double openPos, double closedPos) {
        if (s == null) return;
        if (open) servoArmed[i] = true;
        if (servoArmed[i]) s.setPosition(open ? openPos : closedPos);
    }
```
- **Line 473** — Missing servo → nothing.
- **Line 474** — Any "open" command arms the servo permanently for this run.
- **Line 475** — Until armed, **no signal is ever sent** — the startup safety: a wrong saved value can't slam a servo the instant the program starts.

## Lines 479–504 — show(): the dashboard

```java
    private void show() {
        telemetry.addData("Drive mode", fieldCentric ? "field-centric (press right stick to switch)" : "robot-centric (press right stick to switch)");
```
- **Line 480** — Line 1: which drive mode, always with the switch hint.

```java
        if (tailFirst) telemetry.addLine(fieldCentric ? "!!! Back-is-front is ON, but it does nothing in field-centric (press left stick to turn off)" : ">>> BACK IS FRONT (press left stick to go back to normal) <<<");
```
- **Line 481** — Back-is-front banner — including the honest warning that it's ignored in field-centric (where "front" has no meaning).

```java
        if (fieldCentric) telemetry.addData("Heading", "%.1f deg  (%s, gamepad1 back to reset)", Math.toDegrees(heading()),
                USE_PP_HEADING ? "Pinpoint" : (imu == null ? "imu not found!" : "imu"));
```
- **Lines 482–483** — In field-centric: the current heading, its source, and how to re-zero.

```java
        telemetry.addData("intake", unjam == 1 ? "B: backing up" : unjam == 2 ? "B: pushing in" : (intakeOn ? "on (A = off)" : "off (A = on)"));
```
- **Line 484** — Intake state, including which unjam phase is running.

```java
        telemetry.addData("Flywheels", "%s", flyOn ? "on (X = off)" : "off, coasting (X = on)");
        telemetry.addData("NECTAR rpm", flyLine(flyN, targetN()));
        telemetry.addData("POLLEN rpm", flyLine(flyP, targetP()));
```
- **Lines 485–487** — Flywheels on/off, then each wheel's target vs actual in rpm.

```java
        if (flyAlarm != null) telemetry.addLine("!!! " + flyAlarm);
        else if (flyOn) telemetry.addLine(flyReady() ? ">>> AT SPEED - ready to feed (bumpers / right trigger burst) <<<" : "Speeding up...");
```
- **Line 488** — The stall alarm (loud) takes priority…
- **Line 489** — …otherwise the at-speed banner or "Speeding up…".

```java
        boolean holdL = cur1.left_bumper || cur2.left_bumper, holdR = cur1.right_bumper || cur2.right_bumper;
        telemetry.addData("Gates", "left(NECTAR) %s | right(POLLEN) %s%s",
                holdL && !latchL ? "waiting for speed" : gateState(0, gateL, latchL || burstOpen),
                holdR && !latchR ? "waiting for speed" : gateState(1, gateR, latchR || burstOpen),
                (cur1.right_trigger > 0.5 || cur2.right_trigger > 0.5) ? " | burst shooting" : "");
```
- **Line 490** — Re-read the bumpers (for display).
- **Lines 491–494** — Gate status each: "waiting for speed" while held-but-not-yet-open, else open/closed/not-found/not-moved; plus the "burst shooting" tag while the trigger is held.

```java
        telemetry.addData("NECTAR shots", "%d shots | last %s", shotN.count, shotN.last);
        telemetry.addData("POLLEN shots", "%d shots | last %s", shotP.count, shotP.last);
```
- **Lines 495–496** — The shot counters and each last shot's summary.

```java
        double v = voltage();
        double s = matchT.seconds();
        telemetry.addData("Battery / time", "%.2f V%s | used %d:%02d (ends at 2:00)", v, v > 0 && v < LOW_VOLT ? " LOW VOLTAGE!" : "", (int) s / 60, (int) s % 60);
```
- **Lines 497–499** — Battery (with the <12 V warning) and elapsed time as m:ss with the 2:00 reminder.

```java
        if (pp != null) telemetry.addData("Pinpoint", "x %.0f  y %.0f mm  heading %.1f deg", pp.getPosition().getX(DistanceUnit.MM),
                pp.getPosition().getY(DistanceUnit.MM), pp.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("Found", found());
        telemetry.update();
    }
```
- **Lines 500–501** — Live odometry readout (useful to correlate with the field).
- **Line 502** — The missing-devices report.
- **Line 503** — Send it all.

## Lines 506–523 — display helpers

```java
    private String gateState(int i, Servo s, boolean open) {
        if (s == null) return "not found";
        if (!servoArmed[i]) return "not moved yet";
        return open ? "open" : "closed";
    }
```
- **Lines 506–510** — Gate status text, honest about missing hardware and never-commanded servos.

```java
    private String flyLine(DcMotorEx m, double tps) {
        if (m == null) return "motor not found";
        return String.format("target %.0f  actual %.0f", tps * 60 / TICKS_PER_REV, m.getVelocity() * 60 / TICKS_PER_REV);
    }
```
- **Lines 512–515** — "target X actual Y" in rpm.

```java
    private String found() {
        StringBuilder miss = new StringBuilder();
        String[] n = {"lf", "lb", "rf", "rb", "flywheelNectar", "flywheelPollen", "intake", "sorterGateL", "sorterGateR", "sorterEndL", "sorterEndR", "imu", "pp"};
        Object[] d = {lf, lb, rf, rb, flyN, flyP, intake, gateL, gateR, endL, endR, imu, pp};
        for (int i = 0; i < n.length; i++) if (d[i] == null) miss.append(n[i]).append(' ');
        return miss.length() == 0 ? "all" : "missing " + miss;
    }
```
- **Lines 517–523** — Walk all 13 devices in parallel arrays (names / references); collect every name whose reference is still null; report "all" or "missing …". This is the one-glance wiring check on the competition field.

## Lines 525–562 — the low-level helpers

```java
    private DcMotorEx motor(String name, boolean reversed, boolean brake) {
        try {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
            m.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
            return m;
        } catch (Exception e) { return null; }
    }
```
- **Lines 526–534** — The drive/intake motor factory: by-name lookup, direction, plain power mode, BRAKE or FLOAT, null on failure. (Drive wheels: encoders unused — odometry is the Pinpoint's job.)

```java
    private DcMotorEx flywheel(String name, boolean reversed) {
        try {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
            m.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(FLY_P, FLY_I, FLY_D, FLY_F));
            return m;
        } catch (Exception e) { return null; }
    }
```
- **Lines 536–546** — The flywheel factory: same, but with encoder reset, **velocity control mode**, FLOAT, and the PIDF from line 126.

```java
    private Servo servo(String name) {
        try { return hardwareMap.get(Servo.class, name); } catch (Exception e) { return null; }
    }
```
- **Lines 548–550** — Servo lookup only — no position command, ever, at startup.

```java
    private static double ramp(double cur, double target, double step) {
        if (target > cur) return Math.min(target, cur + step);
        return target;   // slowing down: give the target right away (the velocity control slows it down by itself)
    }
```
- **Lines 552–555** — Ramp up at most `step` per call; ramp **down** immediately (the velocity controller's own braking is enough — only spin-up needed protecting).

```java
    private static void setPower(DcMotorEx m, double p) { if (m != null) m.setPower(p); }
    private static double clip(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
```
- **Line 557** — Null-safe setPower.
- **Line 558** — Clamp helper.

```java
    private void stopAll() {
        for (DcMotorEx m : new DcMotorEx[]{lf, lb, rf, rb, flyN, flyP, intake}) setPower(m, 0);
    }
}
```
- **Lines 560–562** — End of program: every motor 0 (null-safe), and the class's closing brace.

---

## Evaluation

**Strengths**
- Professional-grade defensive coding: every device can vanish without crashing; `found()` tells you in one line; missing hardware counts as "at speed" so it never deadlocks a human.
- The flywheel stack is genuinely well-engineered: slow ramp (coupler protection), coast-not-brake on stop, a continuous-stall guard with rumble + on-screen diagnosis, and per-run trims that can't silently become permanent.
- Every "at speed / waiting / burst" interaction was tuned from real 11-12 findings (the ±3 % starvation and its GATE_WAIT_S fix is documented at line 450).
- Full-gamepad cur/prev snapshotting is the cleanest edge-detection pattern in the SDK; `pressed()` gives every mechanism both-gamepad control in one line.
- The shot CSV turns practice into data — target rpm, dip depth, recovery time, voltage, match time per shot.

**Watch-outs (not changed — worth knowing)**
- `matchSpeed` is permanently false (Y removed), so `NECTAR_MATCH_TPS`/`POLLEN_MATCH_TPS`, the `loadFly()`/`saveFly()` file, and the `matchSpeed` branch are currently dead code — kept deliberately per the comments.
- `UNJAM_TAP_S` (line 364) is unused — a leftover from the removed hold-to-spit behaviour.
- `HUB_LOGO`/`HUB_USB` are guesses; field-centric uses the IMU corrected by them. If field-centric drives wrong when the robot is turned, that's the first suspect (or set `USE_PP_HEADING = true`).
- The bumper gate latch opens instantly when flywheels are OFF — correct for clearing balls, but remember it in a match: feeding with flywheels off just drops balls at the wheel.
- Loop does a lot per cycle (4 ServoStore map lookups, full telemetry rebuild) — comfortably fine on a Rev Hub, just don't pile much more into it.
