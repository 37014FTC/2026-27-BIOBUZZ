package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Tank drive teleop for the mecanum robot.
 *
 * Controls (gamepad 1):
 *   Left stick Y  -> left wheels     (tank drive: sticks = tracks)
 *   Right stick Y -> right wheels
 *   Right trigger -> slide RIGHT     (mecanum party trick)
 *   Left trigger  -> slide LEFT
 *   Hold either bumper -> SLOW MODE (40% power) for precision
 *
 * Raw motor control — no Pedro/follower involved, so tuning state
 * doesn't affect teleop. Brake mode = crisp stops when sticks release.
 */
@TeleOp(name="Kingsley_TankTeleop")
public class TankTeleop extends LinearOpMode {

    static final double SLOW_MODE_FACTOR = 0.4;
    static final double MAX_POWER = 0.8;   // full stick = 80% voltage cap

    private DcMotor frontLeft, backLeft, frontRight, backRight;

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "lf");
        backLeft   = hardwareMap.get(DcMotor.class, "lb");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backRight  = hardwareMap.get(DcMotor.class, "rb");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "Tank: sticks = tracks, triggers = slide, bumpers = slow.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Tank: each stick drives its side (stick up = negative Y, so negate)
            double left  = -gamepad1.left_stick_y;
            double right = -gamepad1.right_stick_y;

            // Strafe: triggers (analog — half trigger = half speed slide)
            double strafe = gamepad1.right_trigger - gamepad1.left_trigger;

            // Slow mode while holding a bumper
            boolean slow = gamepad1.right_bumper || gamepad1.left_bumper;
            if (slow) {
                left   *= SLOW_MODE_FACTOR;
                right  *= SLOW_MODE_FACTOR;
                strafe *= SLOW_MODE_FACTOR;
            }

            // Mix: tank powers + the mecanum criss-cross for strafe
            // (strafe right = FL+, BL-, FR-, BR+ — the cheat-sheet row)
            frontLeft.setPower(scale(left + strafe));
            backLeft.setPower(scale(left - strafe));
            frontRight.setPower(scale(right - strafe));
            backRight.setPower(scale(right + strafe));

            telemetry.addData("mode", slow ? "SLOW" : "FULL");
            telemetry.addData("left / right", "%.2f / %.2f", left, right);
            telemetry.addData("strafe", "%.2f", strafe);
            telemetry.update();
        }
    }

    // Clamp to +-1 first, THEN scale — so combined inputs cap at MAX_POWER
    // instead of clipping back up to 100%.
    private double scale(double p) {
        return Math.max(-1.0, Math.min(1.0, p)) * MAX_POWER;
    }
}
