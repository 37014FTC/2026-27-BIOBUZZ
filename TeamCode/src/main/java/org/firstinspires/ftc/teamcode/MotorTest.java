package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Wheel identification test — RUN WITH THE WHEELS OFF THE GROUND.
 *
 * Hold a button on gamepad 1 to run ONE motor forward at 30% power:
 *   X = lf (should spin the FRONT-LEFT wheel, pushing the robot forward)
 *   A = lb (BACK-LEFT, forward)
 *   Y = rf (FRONT-RIGHT, forward)
 *   B = rb (BACK-RIGHT, forward)
 *
 * For each button, exactly one wheel should spin, the physically correct
 * one, in the direction that would push the robot forward.
 *   - Wrong wheel moves?  The config name is on the wrong port — fix in
 *     Configure Robot by renaming the motors on their correct ports.
 *   - Right wheel, backward? That motor needs its direction flipped in
 *     code (DiagonalNW and the Pedro MecanumConfig).
 */
@TeleOp(name="Kingsley_MotorTest")
public class MotorTest extends LinearOpMode {

    static final double TEST_POWER = 0.3;

    private DcMotor lf, lb, rf, rb;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");

        telemetry.addData(">", "Wheels UP! Hold X/A/Y/B to test one motor.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            lf.setPower(gamepad1.x ? TEST_POWER : 0);
            lb.setPower(gamepad1.a ? TEST_POWER : 0);
            rf.setPower(gamepad1.y ? TEST_POWER : 0);
            rb.setPower(gamepad1.b ? TEST_POWER : 0);

            String active =
                    (gamepad1.x ? "lf (front-left) " : "") +
                    (gamepad1.a ? "lb (back-left) " : "") +
                    (gamepad1.y ? "rf (front-right) " : "") +
                    (gamepad1.b ? "rb (back-right)" : "");
            telemetry.addData("running", active.isEmpty() ? "(none — hold a button)" : active);
            telemetry.update();
        }
    }
}
