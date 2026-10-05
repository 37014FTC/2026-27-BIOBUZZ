package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/*
 * Distance ruler: push the robot by hand from point A to point B and
 * read the straight-line distance on the Driver Station.
 *
 * Reads position from the goBILDA Pinpoint, which fuses the two dead
 * wheels with its IMU — so the reading survives small amounts of
 * rotation while pushing (no more "don't turn the robot" rule).
 *
 * How to use:
 *   1. Place robot at point A, select this OpMode, press INIT, then START.
 *   2. Push the robot to point B.
 *   3. Read "DISTANCE A to B" on the Driver Station.
 *   4. Press A on gamepad 1 any time to re-zero for a new measurement.
 */
@TeleOp(name="Kingsley_OdometryRuler")
public class DistanceRuler extends LinearOpMode {

    // Pod offsets from the robot's center, in mm (measured on our U-shape
    // chassis — also recorded in pedro/Constants.java).
    //   X pod (parallel, measures forward/back): sideways offset —
    //       LEFT of center is positive, RIGHT is negative.
    //   Y pod (perpendicular, measures strafe): fore/aft offset —
    //       FORWARD of center is positive, BEHIND is negative.
    static final double X_POD_OFFSET_MM = -50;  // X pod is RIGHT of center
    static final double Y_POD_OFFSET_MM = -30;  // Y pod is BEHIND center

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        // "pinpoint" must be the name of the goBILDA Pinpoint I2C device
        // in your robot configuration.
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");

        // Tell the Pinpoint where the pods sit so it can subtract the
        // phantom rolling caused by rotation
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);

        // Which goBILDA pods do you have? Change to goBILDA_SWINGARM_POD
        // if that's what's on your robot.
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // X pod should increase when pushed forward; Y pod should increase
        // when pushed LEFT. Flip to REVERSED if a check below fails.
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Zero position and recalibrate the IMU — robot must be STILL for this
        pinpoint.resetPosAndIMU();

        telemetry.addData(">", "Place at point A, press START, push to B.");
        telemetry.addData(">", "Press A on gamepad 1 to re-zero.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                // Re-zero position only (keeps the IMU calibration)
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }

            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double distance = Math.hypot(x, y); // straight line from the zero point

            telemetry.addData("X", "%.1f in", x);
            telemetry.addData("Y", "%.1f in", y);
            telemetry.addData("Heading", "%.1f deg", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Turn rate", "%.1f deg/s",
                    Math.toDegrees(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)));
            telemetry.addData("DISTANCE A to B", "%.1f in", distance);
            telemetry.update();
        }
    }
}
