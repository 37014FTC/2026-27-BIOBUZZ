package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * QUALIFIER AUTO: forward -> strafe -> forward, then park.
 *
 * Deliberately DEAD-SIMPLE on event day: raw timed motor control, no
 * odometry, no follower — nothing that can mis-navigate or oscillate.
 * Tune the six constants at the venue with a tape measure, redeploy
 * between matches if needed (build is fast).
 *
 * Two versions:
 *   37014_AutoParkLeft  — strafes LEFT between the two forward legs
 *   37014_AutoParkRight — strafes RIGHT
 *
 * Motor names match the rest of the codebase: lf lb rf rb,
 * left side reversed. Wheels-up test first, per the rules of the road.
 */
public class AutoPark extends LinearOpMode {

    // -------- tune these at the venue (seconds / power) --------
    static final double FORWARD_1_S = 1.5;   // first forward leg
    static final double STRAFE_S    = 1.5;   // sideways leg
    static final double FORWARD_2_S = 1.2;   // final forward leg (park)
    static final double DRIVE_PWR   = 0.5;   // forward speed
    static final double STRAFE_PWR  = 0.4;   // sideways speed (slower = truer)
    // ------------------------------------------------------------

    // set by the subclasses: +1 = strafe right, -1 = strafe left
    final double strafeDir;

    private DcMotor lf, lb, rf, rb;
    private final ElapsedTime runtime = new ElapsedTime();

    AutoPark(double strafeDir) {
        this.strafeDir = strafeDir;
    }

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "AutoPark: forward %ss, strafe %s (%s), forward %ss",
                FORWARD_1_S, STRAFE_S, strafeDir > 0 ? "RIGHT" : "LEFT", FORWARD_2_S);
        telemetry.update();
        waitForStart();

        // 1) forward
        drive(0, DRIVE_PWR, FORWARD_1_S);

        // 2) sideways
        drive(strafeDir * STRAFE_PWR, 0, STRAFE_S);

        // 3) forward to park
        drive(0, DRIVE_PWR, FORWARD_2_S);

        stopAll();
        telemetry.addData(">", "PARKED.");
        telemetry.update();
        while (opModeIsActive()) idle();
    }

    // lateral: + = slide right; axial: + = forward (cheat-sheet mix)
    private void drive(double lateral, double axial, double seconds) {
        lf.setPower(axial + lateral);
        lb.setPower(axial - lateral);
        rf.setPower(axial - lateral);
        rb.setPower(axial + lateral);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) idle();
        stopAll();
        sleep(150);   // small settle pause between legs
    }

    private void stopAll() {
        lf.setPower(0); lb.setPower(0); rf.setPower(0); rb.setPower(0);
    }

    // ---- the two selectable variants ----

    @Autonomous(name = "37014_AutoParkLeft")
    public static class Left extends AutoPark {
        public Left() { super(-1); }
    }

    @Autonomous(name = "37014_AutoParkRight")
    public static class Right extends AutoPark {
        public Right() { super(+1); }
    }
}
