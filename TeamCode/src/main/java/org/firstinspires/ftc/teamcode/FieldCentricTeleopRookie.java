package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * ROOKIE version of FieldCentricTeleop — identical field-centric controls,
 * capped at 80% power.
 *
 *   Left stick    -> move relative to the FIELD (frame locked at INIT)
 *   Right stick X -> turn
 *   Hold A        -> slow mode
 *
 * Face the robot "north" and keep it STILL during INIT.
 */
@TeleOp(name="37014_FieldRookie")
public class FieldCentricTeleopRookie extends LinearOpMode {

    static final double SLOW_MODE_FACTOR = 0.4;
    static final double MAX_POWER = 0.8;   // rookie cap

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private GoBildaPinpointDriver pinpoint;

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

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        pinpoint.setOffsets(-50, -30, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        telemetry.addData(">", "ROOKIE field-centric. Face north, keep still, START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double fieldForward = -gamepad1.left_stick_y;
            double fieldRight   =  gamepad1.left_stick_x;

            pinpoint.update();
            double h = pinpoint.getHeading(AngleUnit.RADIANS);

            double axial   = fieldForward * Math.cos(h) - fieldRight * Math.sin(h);
            double lateral = fieldForward * Math.sin(h) + fieldRight * Math.cos(h);
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
            telemetry.addData("heading", "%.0f deg", Math.toDegrees(h));
            telemetry.update();
        }
    }

    private double scale(double p) {
        return Math.max(-1.0, Math.min(1.0, p)) * MAX_POWER;
    }
}
