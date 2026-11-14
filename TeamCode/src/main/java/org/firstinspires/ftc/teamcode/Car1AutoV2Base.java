package org.firstinspires.ftc.teamcode;

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

/**
 * Car 1 autonomous, version 2 (11-12). Two OpModes use this base class: "Car1 Auto Shoot+Park" and "Car1 Auto Park Only".
 * Red and blue use the same program: the field is point-symmetric about its center (based on the official top view of
 * the field; sizes not checked one by one), so from the robot's point of view the moves are the same.
 *
 * ===== Before the start (after INIT, before PLAY) choose with gamepad1 =====
 *   dpad up / down   seconds to wait at the start (0-10)      dpad right / left   how many balls to shoot (0-4, not used by Park Only)
 *   X                red / blue (display only, does not change the moves)
 *   Y                small-space test run: path shrunk to 15%, flywheel does not spin (servos still move)
 *   After INIT do not move the robot (the Pinpoint is reset to zero).
 *
 * ===== Coordinates (Pinpoint, start position is the origin) =====  x forward + (front of robot), y left +,
 *   angle counter-clockwise +. Units: mm.
 *
 * ===== Not yet run on the robot / must be checked on a real field =====
 *   Waypoints are estimated from the field drawing (one floor tile = 609.6 mm); the gap between the 4 shots and how long
 *   the gate opens; the closed-loop numbers were tried on a wooden floor (PP Test step 7), not yet on field tiles.
 */
public abstract class Car1AutoV2Base extends LinearOpMode {

    // ---------------- Drive / Pinpoint (same as Car1TeleOp and PP Test; 11-12 PP Test on robot: directions correct) ----------------
    static final boolean LEFT_REVERSED = true, RIGHT_REVERSED = false;
    static final double MAX_P = 0.4, MIN_P = 0.08, KP_XY = 0.004, KP_H = 0.9;
    static final double TOL_MM = 25, TOL_DEG = 3, SEG_TIMEOUT_S = 6;
    static final double AUTO_END_S = 29.0;   // keep 1 second spare

    // ---------------- Shooting ----------------
    static final double TICKS_PER_REV = 28;
    static final double POLLEN_RPM = 3900;          // 11-15 field tuning: 5500 way too much; wall-start auto sweet spot 3900 (teleop shoots 3750 — different distance)
    static final boolean POLLEN_REVERSED = false;
    static final double FLY_P = 10, FLY_I = 1, FLY_D = 0, FLY_F = 32767.0 / 2800;   // 2800 = TICKS/SEC of the ~6000 rpm motor (6000 rpm * 28 ticks / 60), NOT rpm — same as teleop's 32767/FLY_MAX_TPS. (Verified 11-14: an attempted "fix" to /6000 was wrong and reverted — units, not motor, were the confusion.)
    static final double AT_SPEED_BAND = 0.03;
    static final double SPINUP_MAX_S = 3.0;         // wait at most 3 seconds to get to speed; shoot anyway if it doesn't
    static final long GATE_PULSE_MS = 350;          // how long the gate opens to let out one ball (not tested)
    static final double SHOT_GAP_MAX_S = 1.5;       // max wait between two shots to get back to speed
    static final double SHOOT_DEADLINE_S = 20.0;    // if not done shooting after 20 seconds, stop shooting and go park
    static final double GATE_R_OPEN = 0.46, GATE_R_CLOSED = 0.56;   // default values; if car1_servo.txt on the Hub has saved numbers, those are used

    static final String LOG = "/sdcard/FIRST/car1_auto.txt";

    protected DcMotorEx lf, lb, rf, rb, flyP;
    protected Servo gateR;
    protected GoBildaPinpointDriver pp;
    protected double delayS = 0;
    protected int balls = 4;
    protected boolean red = true, smallTest = false;
    protected final ElapsedTime match = new ElapsedTime();
    private Pose2D good = null;
    private int badReads = 0;

    /** Subclass: waypoints {x, y, angle deg}, all in mm on the REAL field. */
    protected abstract double[][] path();
    protected abstract boolean shoots();
    protected abstract String title();

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        lf = motor("lf", LEFT_REVERSED); lb = motor("lb", LEFT_REVERSED);
        rf = motor("rf", RIGHT_REVERSED); rb = motor("rb", RIGHT_REVERSED);
        if (shoots()) {
            try {
                flyP = hardwareMap.get(DcMotorEx.class, "flywheelPollen");
                flyP.setDirection(POLLEN_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
                flyP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                flyP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                flyP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                flyP.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(FLY_P, FLY_I, FLY_D, FLY_F));
            } catch (Exception e) { flyP = null; }
            try { gateR = hardwareMap.get(Servo.class, "sorterGateR"); } catch (Exception e) { gateR = null; }
        } else balls = 0;
        try {
            pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
            pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            pp.setOffsets(15, 55, DistanceUnit.MM);
            pp.resetPosAndIMU();
        } catch (Exception e) { pp = null; }
        ServoStore.load();
        if (gateR != null) gateR.setPosition(gateClosed());   // gate closed at the start (holds the preloaded balls)

        boolean pu = false, pd = false, pl = false, pr = false, px = false, py = false;
        while (opModeInInit()) {
            if (pp != null) pp.update();
            if (gamepad1.dpad_up && !pu) delayS = Math.min(10, delayS + 1);
            if (gamepad1.dpad_down && !pd) delayS = Math.max(0, delayS - 1);
            if (shoots() && gamepad1.dpad_right && !pr) balls = Math.min(4, balls + 1);
            if (shoots() && gamepad1.dpad_left && !pl) balls = Math.max(0, balls - 1);
            if (gamepad1.x && !px) red = !red;
            if (gamepad1.y && !py) smallTest = !smallTest;
            pu = gamepad1.dpad_up; pd = gamepad1.dpad_down; pl = gamepad1.dpad_left; pr = gamepad1.dpad_right; px = gamepad1.x; py = gamepad1.y;

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
            sleep(30);
        }
        if (!opModeIsActive()) return;
        match.reset();
        log(String.format(Locale.US, "==== %s %s wait %.0f s shoot %d %s", new java.util.Date(), title(), delayS, balls, smallTest ? "small-space test run" : "real"));
        if (pp == null) { log("pp not found, not moving"); return; }

        while (opModeIsActive() && match.seconds() < delayS) { readPose(); idle(); }

        if (shoots() && balls > 0) shoot();
        if (flyP != null) flyP.setPower(0);   // coast to stop

        double k = smallTest ? 0.15 : 1.0;
        for (double[] w : path()) {
            if (!opModeIsActive() || match.seconds() > AUTO_END_S) break;
            boolean ok = goTo(w[0] * k, w[1] * k, w[2]);
            log(String.format(Locale.US, "to (%.0f, %.0f, %.0f deg) %s  %.1f s", w[0] * k, w[1] * k, w[2], ok ? "reached" : "NOT reached", match.seconds()));
        }
        stopDrive();
        log(String.format(Locale.US, "end %.1f s, position %s, bad reads %d", match.seconds(), poseStr(readPose()), badReads));
        while (opModeIsActive()) { readPose(); telemetry.addData("Done", "%.1f s  %s", match.seconds(), poseStr(good)); telemetry.update(); }
    }

    // ---------------- Shooting ----------------
    private void shoot() {
        double tps = POLLEN_RPM * TICKS_PER_REV / 60;
        if (flyP != null && !smallTest) flyP.setVelocity(tps);
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < SPINUP_MAX_S && !smallTest && !atSpeed(tps)) { readPose(); status("speeding up"); }
        for (int i = 1; i <= balls && opModeIsActive(); i++) {
            if (match.seconds() > SHOOT_DEADLINE_S) { log("past 20 seconds, skipping the rest of the shots"); break; }
            if (gateR != null) gateR.setPosition(gateOpen());
            sleep(GATE_PULSE_MS);
            if (gateR != null) gateR.setPosition(gateClosed());
            log(String.format(Locale.US, "shot %d at %.1f s, flywheel %.0f rpm", i, match.seconds(), flyP == null ? 0 : flyP.getVelocity() * 60 / TICKS_PER_REV));
            t.reset();
            while (opModeIsActive() && t.seconds() < SHOT_GAP_MAX_S && (smallTest ? t.seconds() < 0.8 : !atSpeed(tps) || t.seconds() < 0.4)) { readPose(); status("after shot " + i + ", getting back to speed"); }
        }
    }

    private boolean atSpeed(double tps) { return flyP == null || Math.abs(flyP.getVelocity() - tps) <= AT_SPEED_BAND * tps; }
    private double gateOpen() { return ServoStore.get("sorterGateR", true, GATE_R_OPEN); }
    private double gateClosed() { return ServoStore.get("sorterGateR", false, GATE_R_CLOSED); }

    // ---------------- Drive to a point (closed loop, same method as PP Test step 7) ----------------
    private boolean goTo(double tx, double ty, double tdeg) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < SEG_TIMEOUT_S && match.seconds() < AUTO_END_S) {
            Pose2D p = readPose();
            double cx = p.getX(DistanceUnit.MM), cy = p.getY(DistanceUnit.MM), ch = p.getHeading(AngleUnit.RADIANS);
            double ex = tx - cx, ey = ty - cy, eh = wrap(Math.toRadians(tdeg) - ch);
            if (Math.hypot(ex, ey) < TOL_MM && Math.abs(Math.toDegrees(eh)) < TOL_DEG) { stopDrive(); return true; }
            double fwd = Math.cos(ch) * ex + Math.sin(ch) * ey;
            double left = -Math.sin(ch) * ex + Math.cos(ch) * ey;
            // Scale both by the same amount, so the direction stays the same when driving diagonally
            double f = KP_XY * fwd, l = KP_XY * left, m = Math.hypot(f, l);
            if (m > MAX_P) { f *= MAX_P / m; l *= MAX_P / m; }
            else if (m > 0.001 && m < MIN_P) { f *= MIN_P / m; l *= MIN_P / m; }
            double r = Math.max(-MAX_P, Math.min(MAX_P, KP_H * eh));
            drive(f, -l, -r);
            status(String.format(Locale.US, "going to (%.0f, %.0f)  error %.0f mm", tx, ty, Math.hypot(ex, ey)));
        }
        stopDrive();
        return false;
    }

    private void status(String s) {
        telemetry.addData("Auto", "%s  %.1f s", s, match.seconds());
        telemetry.addData("pp", "%s  bad reads %d", poseStr(good), badReads);
        telemetry.update();
    }

    // ---------------- Read pp (filter out bad reads, same as PP Test) ----------------
    protected Pose2D readPose() {
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

    private static String poseStr(Pose2D p) {
        if (p == null) return "-";
        return String.format(Locale.US, "x %.0f y %.0f  %.1f deg", p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM), p.getHeading(AngleUnit.DEGREES));
    }

    // ---------------- Drive ----------------
    private void drive(double y, double x, double r) {
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        setP(lf, (y + x + r) / d); setP(lb, (y - x + r) / d);
        setP(rf, (y - x - r) / d); setP(rb, (y + x - r) / d);
    }

    private void stopDrive() { setP(lf, 0); setP(lb, 0); setP(rf, 0); setP(rb, 0); }

    private DcMotorEx motor(String n, boolean rev) {
        try {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, n);
            m.setDirection(rev ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return m;
        } catch (Exception e) { return null; }
    }

    private static void setP(DcMotorEx m, double p) { if (m != null) m.setPower(p); }

    protected static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    protected void log(String s) {
        try (FileWriter w = new FileWriter(LOG, true)) { w.write(s + "\n"); } catch (Exception ignored) { }
    }
}
