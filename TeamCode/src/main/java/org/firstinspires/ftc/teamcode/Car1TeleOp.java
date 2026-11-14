package org.firstinspires.ftc.teamcode;

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

/**
 * Car 1 match TeleOp, version 3 (11-12; version 2 was 11-11). Config names follow the wiring table:
 *   Control Hub: motors 0 lb, 1 lf, 2 rb, 3 rf ; servos 0 sorterGateL, 1 sorterGateR, 2 sorterEndL, 3 sorterEndR ;
 *                I2C bus 0: imu (built in) + pp (Pinpoint), both on the same bus
 *   Expansion Hub: motors 0 flywheelNectar, 1 flywheelPollen, 2 intake
 *   (The program only cares about the NAMES. If a cable is in the wrong port it still works as long as the name is right.
 *    If a device is not in the config / not plugged in, the program does not crash; the screen shows "not found".)
 *
 * ===== Button map =====
 * Mechanism buttons work on BOTH gamepads (one person with gamepad1 can test everything; in a match the operator uses gamepad2):
 *   A               intake on / off
 *   B               back up then push (11-12): reverse 0.25 s -> forward 1 s -> back to whatever on / off it was before.
 *                   Use it when a ball is stuck between the intake and the sorter. It does not spit the ball out.
 *                   (11-12: "hold to spit balls out" was removed)
 *   X               both flywheels on / off (on: ramps up slowly to the target; off: power cut, coast to stop with FLOAT)
 *   Y               (not used) since 11-12 the flywheels have only one speed: NECTAR 3800 / POLLEN 5500 rpm at start
 *   left bumper hold   left gate sorterGateL opens -> NECTAR (big ball) goes into the flywheel; release to close
 *   right bumper hold  right gate sorterGateR opens -> POLLEN (small ball) goes into the flywheel; release to close
 *                   (v3: if the flywheels are on but that wheel is not at speed yet, holding does not open the gate at first;
 *                    it opens by itself when at speed, then stays open until you release. If the flywheels are off it opens
 *                    right away, which is useful to clear balls)
 *   right trigger hold  one-button shooting from against the wall (v3): if the flywheels are off it turns them on;
 *                   when both are at speed -> both gates open for FEED_MS -> close -> wait until back at speed -> open again,
 *                   and keeps shooting until you release
 *   dpad up / down    NECTAR flywheel target +/- 100 rpm (only for this run, not saved; next start goes back to 3800 / 5500)
 *   dpad right / left POLLEN flywheel target +/- 100 rpm
 * gamepad1 only (driver):
 *   left stick         drive (forward / back, left / right)
 *   right stick left/right   turn on the spot
 *   left trigger hold  slow mode
 *   back               reset heading (for field-centric: press it when the front of the robot points away from you)
 *   right stick press  switch robot-centric <-> field-centric (default robot-centric; shown on the first line of the screen)
 *   left stick press   switch "back of robot is the front" <-> normal (only works in robot-centric;
 *                      on: gamepad buzzes 2 times and the screen shows it; off: 1 long buzz)
 * gamepad2 only (operator):
 *   left trigger hold  both end servos sorterEndL / sorterEndR open together (car 1 does not have end ball exits yet,
 *                      so if they are not plugged in nothing happens; in v3 the right trigger is used for one-button shooting)
 *   back               stop all mechanisms (intake off, flywheels off)
 *   After turning on the flywheels / changing speed, the gamepad buzzes once the FIRST time both are at speed
 *   (small ups and downs after that do not buzz again). The screen shows "AT SPEED - ready to feed".
 *
 * ===== Added in v3 (11-12) =====
 *   - Shot log: after the flywheel is at speed, a speed drop of more than 5% counts as one shot; getting back to speed
 *     ends the shot. The screen shows the shot count, lowest rpm, how many % it dropped, and how many ms it took
 *     to get back to speed. When the program ends it is added to /sdcard/FIRST/car1_shots.csv on the Hub
 *     (used to tune the ball path)
 *   - Flywheel speeds (test / match) can be saved in /sdcard/FIRST/car1_fly.txt (loading is turned off right now)
 *   - Screen: battery voltage (warning below 12 V), Pinpoint x / y / heading, TeleOp time used;
 *     at 1:30 and 1:50 the gamepads buzz (30 / 10 seconds left)
 *
 * ===== Safety =====
 *   - Servos do not move at start: no signal is sent to a servo until its button is pressed the first time
 *     (so a wrong OPEN / CLOSED number cannot crash a gate).
 *   - Flywheels take at least 1 second from 0 to full speed (slow ramp, to protect the printed couplers).
 *     When turned off they lose power and coast to stop, no braking.
 *   - Flywheel guard: target > 300 ticks/s but real speed < 25% of target for 1.5 seconds
 *     (encoder not plugged in / wrong direction / jammed) -> power cut automatically, warning on screen, press X to restart.
 *
 * ===== Constants to check on the robot (see the "Robot-tested constants" section below) =====
 *   1. Drive directions LEFT_REVERSED / RIGHT_REVERSED (first use Car1 Hardware Test: push the stick forward,
 *      does each wheel spin forward?)
 *   2. Flywheel directions NECTAR_REVERSED / POLLEN_REVERSED, intake direction INTAKE_REVERSED
 *   3. OPEN / CLOSED for the 4 servos: use Car1 Hardware Test (bumpers +/-0.02) and save the numbers
 *      (the gates were done on 11-11; the end servos are still placeholders 0.3 / 0.7)
 *   4. Flywheel match speeds NECTAR_MATCH_TPS / POLLEN_MATCH_TPS (set after shooting tests); PIDF not tuned yet
 *   5. Field-centric: HUB_LOGO / HUB_USB are which way the Control Hub faces on the robot (not measured);
 *      if you don't want to measure, set USE_PP_HEADING = true to use the Pinpoint instead
 *
 * Old note from the first version: "compiled only, not run on the robot". (Version 3 has since been tested on the robot.)
 */
@TeleOp(name = "Car1 TeleOp", group = "Car1")
public class Car1TeleOp extends LinearOpMode {

    // ---------------- Robot-tested constants ----------------
    static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;       // not measured
    static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;    // not measured
    static final boolean USE_PP_HEADING = false;   // true = field-centric uses the Pinpoint heading

    static final boolean LEFT_REVERSED = true;     // lf lb REVERSE (the two sides are mirror images; a guess)
    static final boolean RIGHT_REVERSED = false;
    static final double STRAFE_FIX = 1.1;          // mecanum strafe correction
    static final double SLOW_SCALE = 0.4;

    static final double TICKS_PER_REV = 28;        // 5203 1:1 motor shaft: 28 ticks per turn
    static final double FLY_MAX_TPS = 2800;        // 6000 rpm
    static final double NECTAR_TEST_TPS = 3800 * TICKS_PER_REV / 60;   // test speed NECTAR 3800 rpm (set 11-12)
    static final double POLLEN_TEST_TPS = 3750 * TICKS_PER_REV / 60;   // POLLEN 3750 rpm (11-15 field tuning: teleop distance sweet spot)
    static final double NECTAR_MATCH_TPS = 0.80 * FLY_MAX_TPS;   // match speed about 4800 rpm, set after shooting tests
    static final double POLLEN_MATCH_TPS = 3750 * TICKS_PER_REV / 60;   // aligned with TEST 11-15: one teleop pollen speed
    static final double FLY_STEP_TPS = 100.0 * TICKS_PER_REV / 60;   // 100 rpm per press
    static final double FLY_RAMP_TPS_PER_SEC = FLY_MAX_TPS;          // 0 -> full speed takes at least 1 second
    static final double AT_SPEED_BAND = 0.03;
    static final double GUARD_MIN_TPS = 300, GUARD_RATIO = 0.25, GUARD_SECONDS = 1.5;
    static final boolean NECTAR_REVERSED = true;    // 11-11 tested on robot: was backwards
    static final boolean POLLEN_REVERSED = false;
    static final double FLY_P = 10, FLY_I = 1, FLY_D = 0, FLY_F = 32767.0 / FLY_MAX_TPS;   // not tuned yet

    static final double INTAKE_POWER = 1.0;
    static final double INTAKE_SPIT_POWER = -1.0;   // 11-12: reverse at full power
    static final boolean INTAKE_REVERSED = true;      // 11-11 tested on robot: was backwards

    // Servo open / closed: first use the numbers Car1 Hardware Test saved on the Hub in /sdcard/FIRST/car1_servo.txt;
    // only if nothing is saved, use the values below
    static final double GATE_L_OPEN = 0.90, GATE_L_CLOSED = 0.78; // 11-11 tested on robot
    static final double GATE_R_OPEN = 0.46, GATE_R_CLOSED = 0.56; // 11-11 tested on robot
    static final double END_L_OPEN = 0.3, END_L_CLOSED = 0.7;     // placeholder
    static final double END_R_OPEN = 0.3, END_R_CLOSED = 0.7;     // placeholder

    // v3: one-button shooting / shot log (not tested yet)
    static final double FEED_MS = 250;        // how long the gates open each time in one-button shooting; test: shortest time that lets out only 1 ball
    static final double FEED_GAP_MS = 150;    // minimum wait after closing before opening again (plus waiting to get back to speed)
    static final double SHOT_DIP = 0.05;      // after at speed, a drop of more than 5% counts as a shot
    static final double LOW_VOLT = 12.0;
    static final String FLY_FILE = "/sdcard/FIRST/car1_fly.txt";
    static final String SHOT_FILE = "/sdcard/FIRST/car1_shots.csv";
    // ------------------------------------------

    private DcMotorEx lf, lb, rf, rb, flyN, flyP, intake;
    private Servo gateL, gateR, endL, endR;
    private final boolean[] servoArmed = new boolean[4];   // no servo signal until the first button press
    private IMU imu;
    private GoBildaPinpointDriver pp;

    private boolean tailFirst = false;   // back of robot is the front
    private boolean fieldCentric = false, intakeOn = false, flyOn = false, matchSpeed = false, rumbled = false;
    private double nectarTest = NECTAR_TEST_TPS, pollenTest = POLLEN_TEST_TPS, nectarMatch = NECTAR_MATCH_TPS, pollenMatch = POLLEN_MATCH_TPS;
    private double cmdN = 0, cmdP = 0;   // current command after the slow ramp
    private String flyAlarm = null;
    private final ElapsedTime guardN = new ElapsedTime(), guardP = new ElapsedTime(), loop = new ElapsedTime();

    private final Gamepad prev1 = new Gamepad(), prev2 = new Gamepad(), cur1 = new Gamepad(), cur2 = new Gamepad();

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

    /** Shot log for one flywheel: after at speed, a drop of more than SHOT_DIP starts a shot; back at speed ends it */
    private final class ShotTrack {
        final String name;
        boolean armed = false, inShot = false;
        double minTps = 0, target = 0;
        final ElapsedTime t = new ElapsedTime();
        int count = 0;
        String last = "none yet";
        ShotTrack(String name) { this.name = name; }

        void reset() { armed = false; inShot = false; }

        void update(DcMotorEx m, double tgt) {
            if (m == null || !flyOn || tgt <= 0) { reset(); return; }
            if (Math.abs(tgt - target) > 1) { target = tgt; reset(); }   // target changed: start again
            double v = m.getVelocity();
            if (!inShot) {
                if (atSpeed(m, tgt)) armed = true;
                else if (armed && v < tgt * (1 - SHOT_DIP)) { inShot = true; minTps = v; t.reset(); }
            } else {
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
    }

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

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

        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
            imu.resetYaw();
        } catch (Exception e) { imu = null; }
        try {
            pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
            pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            pp.setOffsets(15, 55, DistanceUnit.MM);   // numbers from the old chassis, new chassis not measured (only affects position, not heading)
            pp.resetPosAndIMU();
        } catch (Exception e) { pp = null; }

        volts = hardwareMap.getAll(VoltageSensor.class).toArray(new VoltageSensor[0]);
        // 11-12: "always start with these speeds, nothing else" -> saved speeds are not loaded; every start is NECTAR 3800 / POLLEN 5500
        // loadFly();
        ServoStore.load();
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
        guardN.reset(); guardP.reset(); loop.reset(); matchT.reset();

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
    }

    // ---------------- v3: time warnings / voltage / files ----------------
    private void timeWarn() {
        double s = matchT.seconds();
        if (!warn30 && s >= 90) { warn30 = true; gamepad1.rumbleBlips(3); gamepad2.rumbleBlips(3); }
        if (!warn10 && s >= 110) { warn10 = true; gamepad1.rumbleBlips(6); gamepad2.rumbleBlips(6); }
    }

    private double voltage() {
        double v = 0;
        for (VoltageSensor s : volts) { double x = s.getVoltage(); if (x > 0 && (v == 0 || x < v)) v = x; }
        return v;
    }

    private static double rpm(double tps) { return tps * 60 / TICKS_PER_REV; }

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

    private void saveFly() {
        try (FileWriter w = new FileWriter(FLY_FILE, false)) {
            w.write(String.format(Locale.US, "%.1f %.1f %.1f %.1f%n", nectarTest, pollenTest, nectarMatch, pollenMatch));
            flyFileMsg = "saved";
        } catch (Exception e) { flyFileMsg = "save failed " + e.getMessage(); }
    }

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

    // Mechanism buttons: pressed on either gamepad (rising edge)
    private boolean pressed(boolean c1, boolean p1, boolean c2, boolean p2) { return (c1 && !p1) || (c2 && !p2); }

    // ---------------- Drive (gamepad1 only) ----------------
    private void drive() {
        if (cur1.right_stick_button && !prev1.right_stick_button) fieldCentric = !fieldCentric;
        if (cur1.back && !prev1.back) resetHeading();
        // 11-12: press the left stick once -> back of robot is the front (stick forward = drive toward the back,
        // left / right also flip); press again to go back to normal. Makes it easy to aim the back at the goal.
        if (cur1.left_stick_button && !prev1.left_stick_button) {
            tailFirst = !tailFirst;
            if (tailFirst) gamepad1.rumbleBlips(2); else gamepad1.rumble(300);
        }

        double y = -cur1.left_stick_y;
        double x = cur1.left_stick_x * STRAFE_FIX;
        double r = cur1.right_stick_x;
        if (tailFirst && !fieldCentric) { y = -y; x = -x; }   // turning does not flip: seen from above, clockwise is still clockwise
        if (fieldCentric) {
            double h = heading();
            double rx = x * Math.cos(-h) - y * Math.sin(-h);
            double ry = x * Math.sin(-h) + y * Math.cos(-h);
            x = rx; y = ry;
        }
        double scale = cur1.left_trigger > 0.5 ? SLOW_SCALE : 1.0;
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        setPower(lf, (y + x + r) / d * scale);
        setPower(lb, (y - x + r) / d * scale);
        setPower(rf, (y - x - r) / d * scale);
        setPower(rb, (y + x - r) / d * scale);
    }

    private double heading() {
        if (USE_PP_HEADING && pp != null) return pp.getPosition().getHeading(AngleUnit.RADIANS);
        if (imu != null) return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return 0;
    }

    private void resetHeading() {
        if (imu != null) imu.resetYaw();
        if (pp != null) pp.resetPosAndIMU();
    }

    // ---------------- intake ----------------
    // 11-12: when a ball is stuck between the intake and the sorter, back up a little then push it in, without spitting the ball out.
    // Press B: reverse UNJAM_REV_S -> forward UNJAM_FWD_S -> back to the old on / off ("hold to spit" was removed).
    static final double UNJAM_TAP_S = 0.4, UNJAM_REV_S = 0.25, UNJAM_FWD_S = 1.0;
    private int unjam = 0;            // 0 none, 1 backing up, 2 pushing
    private double bDownT = 0, unjamEnd = 0;
    private boolean bWas = false;

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

    // ---------------- Flywheels ----------------
    private void flywheelControl(double dt) {
        if (pressed(cur1.x, prev1.x, cur2.x, prev2.x)) { flyOn = !flyOn; flyAlarm = null; rumbled = false; guardN.reset(); guardP.reset(); }
        // One-button shooting: the moment the right trigger is pressed, turn on the flywheels if they are off
        if (pressed(cur1.right_trigger > 0.5, prev1.right_trigger > 0.5, cur2.right_trigger > 0.5, prev2.right_trigger > 0.5) && !flyOn) {
            flyOn = true; flyAlarm = null; rumbled = false; guardN.reset(); guardP.reset();
        }
        // Y speed switch was removed (11-12: only NECTAR 3800 / POLLEN 5500)
        double sN = 0, sP = 0;
        if (pressed(cur1.dpad_up, prev1.dpad_up, cur2.dpad_up, prev2.dpad_up)) sN += FLY_STEP_TPS;
        if (pressed(cur1.dpad_down, prev1.dpad_down, cur2.dpad_down, prev2.dpad_down)) sN -= FLY_STEP_TPS;
        if (pressed(cur1.dpad_right, prev1.dpad_right, cur2.dpad_right, prev2.dpad_right)) sP += FLY_STEP_TPS;
        if (pressed(cur1.dpad_left, prev1.dpad_left, cur2.dpad_left, prev2.dpad_left)) sP -= FLY_STEP_TPS;
        if (matchSpeed) { nectarMatch = clip(nectarMatch + sN, 0, FLY_MAX_TPS); pollenMatch = clip(pollenMatch + sP, 0, FLY_MAX_TPS); }
        else { nectarTest = clip(nectarTest + sN, 0, FLY_MAX_TPS); pollenTest = clip(pollenTest + sP, 0, FLY_MAX_TPS); }
        // Speeds changed with the dpad only last for this run and are not saved (11-12)

        double step = FLY_RAMP_TPS_PER_SEC * dt;
        cmdN = flyOn ? ramp(cmdN, targetN(), step) : 0;
        cmdP = flyOn ? ramp(cmdP, targetP(), step) : 0;
        spin(flyN, cmdN);
        spin(flyP, cmdP);

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
        // Buzz only the first time it reaches speed after "flywheels on / speed change"; small ups and downs
        // around the target after that do not buzz again (11-12 feedback)
        if (sN != 0 || sP != 0) rumbled = false;
        if (flyReady() && !rumbled) { gamepad1.rumble(250); gamepad2.rumble(250); rumbled = true; }
    }

    private double targetN() { return matchSpeed ? nectarMatch : nectarTest; }
    private double targetP() { return matchSpeed ? pollenMatch : pollenTest; }

    private boolean flyReady() {
        return flyOn && atSpeed(flyN, targetN()) && atSpeed(flyP, targetP());
    }

    private void spin(DcMotorEx m, double tps) {
        if (m == null) return;
        if (flyOn && tps > 0) m.setVelocity(tps);
        else m.setPower(0);   // setPower(0) + FLOAT = coast to stop; setVelocity(0) would brake
    }

    private static boolean stalled(DcMotorEx m, double tps) {
        return m != null && tps > GUARD_MIN_TPS && m.getVelocity() < tps * GUARD_RATIO;
    }

    private static boolean atSpeed(DcMotorEx m, double tps) {
        return m == null || Math.abs(m.getVelocity() - tps) <= AT_SPEED_BAND * tps;
    }

    // ---------------- Sorter servos ----------------
    private void sorterControl() {
        // Bumpers: flywheels off -> opens while held (to clear balls); flywheels on -> opens only when that wheel is
        // at speed, then stays open until released (so it does not pinch a ball that is passing through)
        boolean holdL = cur1.left_bumper || cur2.left_bumper, holdR = cur1.right_bumper || cur2.right_bumper;
        // 11-12 tested on robot: POLLEN at 5500 rpm never got inside +/-3%, so the gate never opened. Changed to:
        // open when at speed, OR open anyway after holding longer than GATE_WAIT_S.
        double now = getRuntime();
        if (holdL && !holdLWas) holdLT = now;
        if (holdR && !holdRWas) holdRT = now;
        holdLWas = holdL; holdRWas = holdR;
        latchL = holdL && (latchL || !flyOn || atSpeed(flyN, targetN()) || now - holdLT >= GATE_WAIT_S);
        latchR = holdR && (latchR || !flyOn || atSpeed(flyP, targetP()) || now - holdRT >= GATE_WAIT_S);

        // One-button shooting: hold the right trigger -> both at speed and at least FEED_GAP_MS since the last close
        // -> both gates open for FEED_MS -> close -> repeat
        boolean burst = cur1.right_trigger > 0.5 || cur2.right_trigger > 0.5;
        if (!burst || !flyOn) burstOpen = false;
        else if (burstOpen) { if (burstT.milliseconds() >= FEED_MS) { burstOpen = false; burstT.reset(); } }
        else if ((flyReady() && burstT.milliseconds() >= FEED_GAP_MS) || burstT.milliseconds() >= FEED_GAP_MS + GATE_WAIT_S * 1000) { burstOpen = true; burstT.reset(); }   // if not at speed, wait at most GATE_WAIT_S extra

        servoCtl(0, gateL, latchL || burstOpen, ServoStore.get("sorterGateL", true, GATE_L_OPEN), ServoStore.get("sorterGateL", false, GATE_L_CLOSED));
        servoCtl(1, gateR, latchR || burstOpen, ServoStore.get("sorterGateR", true, GATE_R_OPEN), ServoStore.get("sorterGateR", false, GATE_R_CLOSED));
        servoCtl(2, endL, cur2.left_trigger > 0.5, ServoStore.get("sorterEndL", true, END_L_OPEN), ServoStore.get("sorterEndL", false, END_L_CLOSED));
        servoCtl(3, endR, cur2.left_trigger > 0.5, ServoStore.get("sorterEndR", true, END_R_OPEN), ServoStore.get("sorterEndR", false, END_R_CLOSED));
    }

    private void servoCtl(int i, Servo s, boolean open, double openPos, double closedPos) {
        if (s == null) return;
        if (open) servoArmed[i] = true;
        if (servoArmed[i]) s.setPosition(open ? openPos : closedPos);
    }

    // ---------------- Screen ----------------
    private void show() {
        telemetry.addData("Drive mode", fieldCentric ? "field-centric (press right stick to switch)" : "robot-centric (press right stick to switch)");
        if (tailFirst) telemetry.addLine(fieldCentric ? "!!! Back-is-front is ON, but it does nothing in field-centric (press left stick to turn off)" : ">>> BACK IS FRONT (press left stick to go back to normal) <<<");
        if (fieldCentric) telemetry.addData("Heading", "%.1f deg  (%s, gamepad1 back to reset)", Math.toDegrees(heading()),
                USE_PP_HEADING ? "Pinpoint" : (imu == null ? "imu not found!" : "imu"));
        telemetry.addData("intake", unjam == 1 ? "B: backing up" : unjam == 2 ? "B: pushing in" : (intakeOn ? "on (A = off)" : "off (A = on)"));
        telemetry.addData("Flywheels", "%s", flyOn ? "on (X = off)" : "off, coasting (X = on)");
        telemetry.addData("NECTAR rpm", flyLine(flyN, targetN()));
        telemetry.addData("POLLEN rpm", flyLine(flyP, targetP()));
        if (flyAlarm != null) telemetry.addLine("!!! " + flyAlarm);
        else if (flyOn) telemetry.addLine(flyReady() ? ">>> AT SPEED - ready to feed (bumpers / right trigger burst) <<<" : "Speeding up...");
        boolean holdL = cur1.left_bumper || cur2.left_bumper, holdR = cur1.right_bumper || cur2.right_bumper;
        telemetry.addData("Gates", "left(NECTAR) %s | right(POLLEN) %s%s",
                holdL && !latchL ? "waiting for speed" : gateState(0, gateL, latchL || burstOpen),
                holdR && !latchR ? "waiting for speed" : gateState(1, gateR, latchR || burstOpen),
                (cur1.right_trigger > 0.5 || cur2.right_trigger > 0.5) ? " | burst shooting" : "");
        telemetry.addData("NECTAR shots", "%d shots | last %s", shotN.count, shotN.last);
        telemetry.addData("POLLEN shots", "%d shots | last %s", shotP.count, shotP.last);
        double v = voltage();
        double s = matchT.seconds();
        telemetry.addData("Battery / time", "%.2f V%s | used %d:%02d (ends at 2:00)", v, v > 0 && v < LOW_VOLT ? " LOW VOLTAGE!" : "", (int) s / 60, (int) s % 60);
        if (pp != null) telemetry.addData("Pinpoint", "x %.0f  y %.0f mm  heading %.1f deg", pp.getPosition().getX(DistanceUnit.MM),
                pp.getPosition().getY(DistanceUnit.MM), pp.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("Found", found());
        telemetry.update();
    }

    private String gateState(int i, Servo s, boolean open) {
        if (s == null) return "not found";
        if (!servoArmed[i]) return "not moved yet";
        return open ? "open" : "closed";
    }

    private String flyLine(DcMotorEx m, double tps) {
        if (m == null) return "motor not found";
        return String.format("target %.0f  actual %.0f", tps * 60 / TICKS_PER_REV, m.getVelocity() * 60 / TICKS_PER_REV);
    }

    private String found() {
        StringBuilder miss = new StringBuilder();
        String[] n = {"lf", "lb", "rf", "rb", "flywheelNectar", "flywheelPollen", "intake", "sorterGateL", "sorterGateR", "sorterEndL", "sorterEndR", "imu", "pp"};
        Object[] d = {lf, lb, rf, rb, flyN, flyP, intake, gateL, gateR, endL, endR, imu, pp};
        for (int i = 0; i < n.length; i++) if (d[i] == null) miss.append(n[i]).append(' ');
        return miss.length() == 0 ? "all" : "missing " + miss;
    }

    // ---------------- Small helpers ----------------
    private DcMotorEx motor(String name, boolean reversed, boolean brake) {
        try {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
            m.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
            return m;
        } catch (Exception e) { return null; }
    }

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

    private Servo servo(String name) {
        try { return hardwareMap.get(Servo.class, name); } catch (Exception e) { return null; }
    }

    private static double ramp(double cur, double target, double step) {
        if (target > cur) return Math.min(target, cur + step);
        return target;   // slowing down: give the target right away (the velocity control slows it down by itself)
    }

    private static void setPower(DcMotorEx m, double p) { if (m != null) m.setPower(p); }
    private static double clip(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    private void stopAll() {
        for (DcMotorEx m : new DcMotorEx[]{lf, lb, rf, rb, flyN, flyP, intake}) setPower(m, 0);
    }
}
