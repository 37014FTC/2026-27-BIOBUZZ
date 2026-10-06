package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Holonomic ("POV") teleop for the mecanum robot.
 *
 * Controls (gamepad 1):
 *   Left stick Y  -> drive forward / backward
 *   Right stick X -> SLIDE left / right (mecanum strafe — robot keeps facing)
 *   Bumpers       -> turn in place (L = counterclockwise, R = clockwise)
 *   Hold A        -> SLOW MODE (40% power)
 *
 * This is the " FPS" scheme: sticks move the robot in any direction
 * without rotating it; bumpers rotate. Raw motor control, no Pedro.
 */
@TeleOp(name="Kingsley_JoystickTeleop")
public class JoystickTeleop extends LinearOpMode {

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

        telemetry.addData(">", "L stick = drive, R stick = slide, bumpers = turn, A = slow.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double axial   = -gamepad1.left_stick_y;   // + = forward
            double lateral =  gamepad1.right_stick_x;  // + = slide right
            double yaw     = (gamepad1.right_bumper ? 0.6 : 0)
                           - (gamepad1.left_bumper  ? 0.6 : 0); // + = clockwise

            if (gamepad1.a) {
                axial   *= SLOW_MODE_FACTOR;
                lateral *= SLOW_MODE_FACTOR;
                yaw     *= SLOW_MODE_FACTOR;
            }

            // The mecanum cheat-sheet mix (same formula as DriveModes):
            // strafe right = FL+, BL-, FR-, BR+; yaw + = spin clockwise
            frontLeft.setPower(scale(axial + lateral + yaw));
            backLeft.setPower(scale(axial - lateral + yaw));
            frontRight.setPower(scale(axial - lateral - yaw));
            backRight.setPower(scale(axial + lateral - yaw));

            telemetry.addData("axial / lateral", "%.2f / %.2f", axial, lateral);
            telemetry.addData("turn", "%.2f%s", yaw, gamepad1.a ? "  SLOW" : "");
            telemetry.update();
        }
    }

    // Clamp to +-1 first, THEN scale — so combined inputs cap at MAX_POWER
    // instead of clipping back up to 100%.
    private double scale(double p) {
        return Math.max(-1.0, Math.min(1.0, p)) * MAX_POWER;
    }
}
