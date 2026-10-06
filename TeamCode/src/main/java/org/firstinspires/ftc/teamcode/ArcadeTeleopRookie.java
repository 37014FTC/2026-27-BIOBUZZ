package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * ROOKIE version of ArcadeTeleop — identical controls, capped at 80%
 * power so new drivers get a forgiving robot.
 *
 *   Left stick    -> move in ANY direction (Y = forward/back, X = strafe)
 *   Right stick X -> turn left / right
 *   Hold A        -> slow mode (40% of the cap)
 */
@TeleOp(name="37014_ArcadeRookie")
public class ArcadeTeleopRookie extends LinearOpMode {

    static final double SLOW_MODE_FACTOR = 0.4;
    static final double MAX_POWER = 0.8;   // rookie cap

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

        telemetry.addData(">", "ROOKIE arcade: L stick move, R stick turn, A slow.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            if (gamepad1.a) {
                axial   *= SLOW_MODE_FACTOR;
                lateral *= SLOW_MODE_FACTOR;
                yaw     *= SLOW_MODE_FACTOR;
            }

            frontLeft.setPower(scale(axial + lateral + yaw));
            backLeft.setPower(scale(axial - lateral + yaw));
            frontRight.setPower(scale(axial - lateral - yaw));
            backRight.setPower(scale(axial + lateral - yaw));

            telemetry.addData("mode", gamepad1.a ? "SLOW" : "ROOKIE (80%)");
            telemetry.update();
        }
    }

    private double scale(double p) {
        return Math.max(-1.0, Math.min(1.0, p)) * MAX_POWER;
    }
}
