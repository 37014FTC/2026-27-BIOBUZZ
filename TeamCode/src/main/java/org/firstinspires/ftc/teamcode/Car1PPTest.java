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

/**
 * Car1 PP Test (11-12): in a small space (1 m x 1 m), check that the Pinpoint pp works and its directions are right,
 * and also check the directions of the 4 drive wheels.
 * Put the robot in the middle of an open floor. Each step moves at most about 20 cm from the start; past 35 cm it stops
 * by itself. Results are written to /sdcard/FIRST/car1_pptest.txt on the Hub.
 *
 * gamepad1:
 *   A        run the next step (the screen shows what the next step is)
 *   B        stop right now (any time)
 *   X / Y    flip the Pinpoint X pod / Y pod direction (only press if steps 1-6 say it is backwards; then it restarts from step 1)
 *   back     reset pp to zero, restart from step 1 (robot must be still)
 *
 * Steps: 1 forward  2 back  3 left  4 right  5 turn left  6 turn right (all low power, open loop; stops at 15 cm / 45 deg or 0.8 s)
 *        7 closed loop: use pp to drive to 20 cm forward -> back to start -> 20 cm left -> back to start -> turn left 90 deg
 *          -> turn back (only allowed if the directions in steps 1-6 are all correct)
 * pp coordinates (goBILDA): x forward is +, y left is +, angle counter-clockwise (turning left) is +.
 */
@TeleOp(name = "Car1 PP Test", group = "Test")
public class Car1PPTest extends LinearOpMode {

    static final boolean LEFT_REVERSED = true, RIGHT_REVERSED = false;   // same as Car1TeleOp
    static final double OPEN_POWER = 0.25, OPEN_TURN = 0.25, OPEN_DIST_MM = 150, OPEN_ANGLE_DEG = 45, OPEN_TIME_S = 0.8;
    static final double MAX_RADIUS_MM = 350;
    static final double KP_XY = 0.004, KP_H = 0.9, MAX_P = 0.3, MIN_P = 0.08, TOL_MM = 15, TOL_DEG = 2, STEP_TIMEOUT_S = 3;
    static final String LOG = "/sdcard/FIRST/car1_pptest.txt";

    private DcMotorEx lf, lb, rf, rb;
    private GoBildaPinpointDriver pp;
    private boolean revX = true, revY = true;   // what the programs use now: both REVERSED (checked on robot 11-12)
    private int step = 1;
    private final String[] result = new String[8];
    private boolean dirX = false, dirY = false, dirH = false;
    private String last = "";
    private boolean pA, pB, pX, pY, pBack;

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        lf = motor("lf", LEFT_REVERSED); lb = motor("lb", LEFT_REVERSED);
        rf = motor("rf", RIGHT_REVERSED); rb = motor("rb", RIGHT_REVERSED);
        try {
            pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
            pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pp.setOffsets(15, 55, DistanceUnit.MM);   // numbers from the old chassis; only affects x/y while turning, not the direction check here
            applyDirs();
        } catch (Exception e) { pp = null; }
        log("==== " + new java.util.Date() + " start, pp " + (pp == null ? "not found" : pp.getDeviceStatus()));

        while (opModeInInit()) {
            if (pp != null) pp.update();
            telemetry.addLine("Car1 PP Test: put robot in the middle of open floor, 50 cm clear all around. Press PLAY, then A for each step, B to stop any time");
            telemetry.addData("pp", pp == null ? "not found! check which I2C port pp is on in config car1" : pp.getDeviceStatus().toString());
            if (pp != null) telemetry.addData("First push the robot by hand", "push forward: x goes up, push left: y goes up, turn left: angle goes up -> %s", pose());
            telemetry.update();
            sleep(50);
        }
        if (pp == null) return;

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
    }

    private void restart(String why) {
        step = 1; for (int i = 0; i < result.length; i++) result[i] = null;
        dirX = dirY = dirH = false; last = why + ", restart from step 1";
        log(why + " -> X pod " + (revX ? "REVERSED" : "FORWARD") + ", Y pod " + (revY ? "REVERSED" : "FORWARD"));
    }

    private void applyDirs() {
        pp.setEncoderDirections(revX ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                revY ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pp.resetPosAndIMU();
        sleep(300);
    }

    // ---------------- Steps ----------------
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

    /** fwd forward +, right right +, cw clockwise +. Stops at 15 cm / 45 deg or 0.8 seconds. */
    private void openLoop(int k, String name, double fwd, double right, double cw) {
        good = null;
        Pose2D p0 = readPose();
        int[] e0 = enc();
        ElapsedTime t = new ElapsedTime();
        String stopWhy = "time up 0.8 s";
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
        settle(400);
        Pose2D p = readPose();
        int[] e = enc();
        double dx = p.getX(DistanceUnit.MM) - p0.getX(DistanceUnit.MM), dy = p.getY(DistanceUnit.MM) - p0.getY(DistanceUnit.MM);
        double dh = Math.toDegrees(wrap(p.getHeading(AngleUnit.RADIANS) - p0.getHeading(AngleUnit.RADIANS)));
        String verdict = judge(k, dx, dy, dh);
        result[k] = String.format(Locale.US, "%d %s: dx %+.0f dy %+.0f mm  turn %+.1f deg  wheels lf %+d lb %+d rf %+d rb %+d  (%s) -> %s",
                k, name, dx, dy, dh, e[0] - e0[0], e[1] - e0[1], e[2] - e0[2], e[3] - e0[3], stopWhy, verdict);
        last = result[k];
        log(result[k]);
    }

    private String judge(int k, double dx, double dy, double dh) {
        double ax = Math.abs(dx), ay = Math.abs(dy);
        boolean moved = Math.hypot(dx, dy) > 30 || Math.abs(dh) > 10;
        if (!moved) return "pp barely changed (robot didn't move? or pp pods not touching the floor / cable not plugged in)";
        switch (k) {
            case 1: if (ax > ay && dx > 0) { dirX = true; return "OK"; } return ax > ay ? "X is backwards (press X to flip)" : "x / y swapped? robot moved forward but pp y is changing";
            case 3: if (ay > ax && dy > 0) { dirY = true; return "OK"; } return ay > ax ? "Y is backwards (press Y to flip)" : "x / y swapped? robot moved left but pp x is changing";
            case 5: if (dh > 0) { dirH = true; return "OK (turning left makes angle go up)"; } return "angle is backwards (Pinpoint mounted upside down? tell the team)";
            default: return "return trip";
        }
    }

    private void closedLoop() {
        double[][] tg = {{200, 0, 0}, {0, 0, 0}, {0, 200, 0}, {0, 0, 0}, {0, 0, 90}, {0, 0, 0}};
        good = null;
        int bad0 = badReads;
        Pose2D o = readPose();
        double ox = o.getX(DistanceUnit.MM), oy = o.getY(DistanceUnit.MM), oh = o.getHeading(AngleUnit.RADIANS);
        StringBuilder sb = new StringBuilder("7 closed loop:");
        for (double[] g : tg) {
            ElapsedTime t = new ElapsedTime();
            double ex = 0, ey = 0, eh = 0;
            boolean ok = false;
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
            sb.append(String.format(Locale.US, "  (%.0f,%.0f,%.0f deg) %s error %.0f mm %.1f deg %.1f s;", g[0], g[1], g[2], ok ? "reached" : "NOT reached", Math.hypot(ex, ey), Math.toDegrees(eh), t.seconds()));
            if (gamepad1.b) sb.append(" [B pressed]");
            if (gamepad1.b || !ok) break;
        }
        sb.append(String.format(Locale.US, "  bad reads %d", badReads - bad0));
        result[7] = sb.toString(); last = result[7]; log(result[7]);
    }

    private double clipMag(double v) {
        double a = Math.min(MAX_P, Math.abs(v));
        if (a > 0.001) a = Math.max(MIN_P, a);
        return Math.copySign(a, v);
    }


    // ---------------- Read pp (filter out bad reads) ----------------
    // 11-12 test: the second closed-loop run stopped after 0.4 s. The log showed the position reading suddenly jumped
    // a lot and set off the 35 cm safety stop.
    // In one loop (about 20 ms) the robot can't move 60 mm or turn 15 deg, so anything bigger is a bad read: throw it
    // away and use the last good value.
    private Pose2D good = null;
    private int badReads = 0;

    /** Stop and wait for the robot to coast to a stop, reading pp the whole time (if we don't read, the coasting distance would look like a bad read). */
    private void settle(long ms) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.milliseconds() < ms) { readPose(); sleep(10); }
    }

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

    // ---------------- Drive ----------------
    private void drive(double y, double x, double r) {
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        setP(lf, (y + x + r) / d); setP(lb, (y - x + r) / d);
        setP(rf, (y - x - r) / d); setP(rb, (y + x - r) / d);
    }

    private void stopDrive() { setP(lf, 0); setP(lb, 0); setP(rf, 0); setP(rb, 0); }

    private int[] enc() {
        DcMotorEx[] m = {lf, lb, rf, rb};
        int[] e = new int[4];
        for (int i = 0; i < 4; i++) e[i] = m[i] == null ? 0 : m[i].getCurrentPosition();
        return e;
    }

    private DcMotorEx motor(String n, boolean rev) {
        try {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, n);
            m.setDirection(rev ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return m;
        } catch (Exception e) { return null; }
    }

    private static void setP(DcMotorEx m, double p) { if (m != null) m.setPower(p); }

    // ---------------- Screen / log ----------------
    private void show() {
        String[] names = {"", "forward", "back", "left", "right", "turn left", "turn right", "closed loop (fwd 20 / back / left 20 / back / turn left 90 / back)"};
        telemetry.addData("Next step (press A)", step <= 7 ? step + " " + names[step] : "all done");
        telemetry.addData("pp", "%s  %s", pp.getDeviceStatus(), pose());
        telemetry.addData("Bad reads (thrown away)", badReads);
        telemetry.addData("pp pod directions", "X %s | Y %s  (X / Y buttons flip)", revX ? "REVERSED" : "FORWARD", revY ? "REVERSED" : "FORWARD");
        telemetry.addLine(last);
        for (int i = 1; i <= 7; i++) if (result[i] != null) telemetry.addLine(result[i]);
        String miss = (lf == null ? "lf " : "") + (lb == null ? "lb " : "") + (rf == null ? "rf " : "") + (rb == null ? "rb " : "");
        if (!miss.isEmpty()) telemetry.addData("Motors not found", miss);
        telemetry.addLine("B stop | back reset and restart");
        telemetry.update();
    }

    private String pose() {
        Pose2D p = pp.getPosition();
        return String.format(Locale.US, "x %.0f  y %.0f mm  angle %.1f deg", p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM), p.getHeading(AngleUnit.DEGREES));
    }

    private static double radius(Pose2D p) { return Math.hypot(p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM)); }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private void log(String s) {
        try (FileWriter w = new FileWriter(LOG, true)) { w.write(s + "\n"); } catch (Exception ignored) { }
    }
}
