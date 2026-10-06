package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Drives forward, backward, then spins in place. Written for a 4-motor
 * mecanum drive. Motor names must match your robot configuration.
 *
 * Forward/back and spinning work like a tank drive (both sides same power
 * to go straight, opposite powers to spin). Mecanum wheels additionally
 * allow strafing (sliding sideways), provided by the strafe() method below.
 */
@Autonomous(name="37014_FBSTest")
public class ForwardBackSpin extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    @Override
    public void runOpMode() {
        // Names must match your robot configuration on the Driver Station
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "lf");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "lb");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backRightDrive  = hardwareMap.get(DcMotor.class, "rb");

        // Flip FORWARD <-> REVERSE on whichever side runs backward on your robot.
        // All four wheels should push the robot forward when set to +power.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        // drive( 0.25,  0.25, 2000);  // forward for 2 seconds
        // drive(-0.25, -0.25, 2000);  // backward for 2 seconds
        drive( 0.25, -0.25, 4000);  // spin clockwise: left wheels forward, right wheels back

        // Mecanum bonus — slide sideways (uncomment to try):
        strafe(0.25, 1500);     // slides right; use -0.5 to slide left
    }

    // Runs the LEFT side at leftPower and the RIGHT side at rightPower,
    // then stops after the given time.
    private void drive(double leftPower, double rightPower, long milliseconds) {
        frontLeftDrive.setPower(leftPower);
        backLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        backRightDrive.setPower(rightPower);

        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < milliseconds) {
            idle();
        }

        stopWheels();
    }

    // Mecanum only: slides the robot sideways. Positive power slides right.
    // Wheels on one diagonal run forward while the other diagonal runs backward.
    private void strafe(double power, long milliseconds) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(-power);
        frontRightDrive.setPower(-power);
        backRightDrive.setPower(power);

        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < milliseconds) {
            idle();
        }

        stopWheels();
    }

    private void stopWheels() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
