package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * FIELD-CENTRIC arcade drive — the scheme most mecanum teams compete with.
 *
 *   Left stick    -> move relative to the FIELD: push away = go "north",
 *                    no matter which way the robot's nose points.
 *   Right stick X -> turn left / right
 *   Hold A        -> slow mode (40%)
 *
 * How it works: the Pinpoint reports the robot's heading; every loop the
 * driver's stick input (a field-frame direction) is rotated into the
 * robot's current frame before the wheel mix. Push diagonally and the
 * robot glides diagonally in the FIELD sense even while facing elsewhere.
 *
 * The field frame is locked when you press INIT (robot must be STILL —
 * the IMU calibrates then). Face the robot "north" (typically toward the
 * field, away from the driver) before initializing.
 */
@TeleOp(name="37014_FieldCentric")
public class FieldCentricTeleop extends LinearOpMode {

    static final double SLOW_MODE_FACTOR = 0.4;
    static final double MAX_POWER = 1.0;   // full stick = full voltage

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

        // Pinpoint — same calibration as our other programs (heading is
        // what we use here; offsets/directions kept for consistency)
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        pinpoint.setOffsets(-50, -30, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();   // robot STILL — this locks "north"

        telemetry.addData(">", "Face robot 'north', keep still, then START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Driver input in FIELD frame (set at INIT)
            double fieldForward = -gamepad1.left_stick_y;  // + = field north
            double fieldRight   =  gamepad1.left_stick_x;   // + = field east

            pinpoint.update();
            double h = pinpoint.getHeading(AngleUnit.RADIANS); // CCW positive

            // Rotate the field-frame command into the robot's CURRENT frame:
            //   axial        = forward along the robot's nose
            //   lateral (+R) = toward the robot's right
            double axial   = fieldForward * Math.cos(h) - fieldRight * Math.sin(h);
            double lateral = fieldForward * Math.sin(h) + fieldRight * Math.cos(h);

            double yaw = gamepad1.right_stick_x;  // + = turn right (CW)

            if (gamepad1.a) {
                axial   *= SLOW_MODE_FACTOR;
                lateral *= SLOW_MODE_FACTOR;
                yaw     *= SLOW_MODE_FACTOR;
            }

            frontLeft.setPower(scale(axial + lateral + yaw));
            backLeft.setPower(scale(axial - lateral + yaw));
            frontRight.setPower(scale(axial - lateral - yaw));
            backRight.setPower(scale(axial + lateral - yaw));

            telemetry.addData("heading", "%.0f deg", Math.toDegrees(h));
            telemetry.addData("field cmd", "%.2f / %.2f", fieldForward, fieldRight);
            telemetry.addData("robot cmd", "%.2f / %.2f / %.2f%s",
                    axial, lateral, yaw, gamepad1.a ? "  SLOW" : "");
            telemetry.update();
        }
    }

    // Clamp to +-1 first, THEN scale — combined inputs cap at MAX_POWER
    private double scale(double p) {
        return Math.max(-1.0, Math.min(1.0, p)) * MAX_POWER;
    }
}
