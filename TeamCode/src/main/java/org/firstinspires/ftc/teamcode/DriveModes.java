package org.firstinspires.ftc.teamcode;
// claude --resume 91ec36b0-e1aa-4146-8fa7-5d2ba3a1f0c8
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * Selectable autonomous routines. Pick one DURING INIT with gamepad 1:
 *
 *   D-pad UP    = Forward, Back, Spin (the timed routine)
 *   D-pad DOWN  = Slide sideways until the touch sensor presses a wall,
 *                 then slide back to the start using odometry
 *   D-pad RIGHT = Diagonal there-and-back (forward-right, then return)
 *   X (press)   = toggle LOOP on/off — repeat the routine until STOP
 *
 * Your selection shows on the Driver Station while in INIT; press START to run.
 *
 * Hardware needed in the config (names in quotes):
 *   motors: front_left_drive, back_left_drive, front_right_drive,
 *           back_right_drive
 *   touch:  "touch" — a REV Touch Sensor on a digital port, mounted on the
 *           side of the robot that will bump the wall while sliding
 *   odometry: "pp" — the goBILDA Pinpoint
 */
@Autonomous(name="37014_Modes")
public class DriveModes extends LinearOpMode {

    // Pinpoint setup — same values as 37014_OdometryRuler
    static final double X_POD_OFFSET_MM = -50;  // X pod is RIGHT of center
    static final double Y_POD_OFFSET_MM = -30;  // Y pod is BEHIND center

    static final double DRIVE_POWER = 0.5;
    static final double SLIDE_POWER = 0.35;

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private TouchSensor touch;
    private GoBildaPinpointDriver pinpoint;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // ---------------- hardware setup ----------------
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        touch = hardwareMap.get(TouchSensor.class, "touch");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // ---------------- mode selection during INIT ----------------
        int mode = 0;              // 0 = drive routine, 1 = slide-until-touch
        boolean loopMode = false;
        boolean xWasDown = false;

        while (opModeInInit()) {
            if (gamepad1.dpad_up)    mode = 0;
            if (gamepad1.dpad_down)  mode = 1;
            if (gamepad1.dpad_right) mode = 2;

            // Toggle loopMode once per X press (edge detection: only count
            // the moment the button goes from up to down)
            boolean xDown = gamepad1.x;
            if (xDown && !xWasDown) loopMode = !loopMode;
            xWasDown = xDown;

            telemetry.addData("Mode",
                    mode == 0 ? "Forward, Back, Spin"
                    : mode == 1 ? "Slide until touch, return"
                    : "Diagonal there-and-back");
            telemetry.addData("Loop", loopMode
                    ? "ON (repeats until STOP)" : "OFF (runs once)");
            telemetry.addLine("D-pad UP/DOWN picks mode, X toggles loop");
            telemetry.update();
        }

        waitForStart();

        do {
            if (mode == 0) {
                forwardBackSpinRoutine();
            } else if (mode == 1) {
                slideUntilTouchRoutine();
            } else {
                diagonalRoutine();
            }
            if (loopMode) sleep(1000); // short pause between repeats
        } while (opModeIsActive() && loopMode);
    }

    // ---------------- routine 0: forward, back, spin ----------------

    private void forwardBackSpinRoutine() {
        drive( 0.5,  0.5, 2000);  // forward 2 s
        drive(-0.5, -0.5, 2000);  // backward 2 s
        drive( 0.5, -0.5, 1500);  // spin clockwise 1.5 s
    }

    // ---------------- routine 2: diagonal there-and-back ----------------

    private void diagonalRoutine() {
        // A diagonal is just forward + strafe at the same time.
        // Pure 45° diagonal = only the FL/BR wheel pair spins!
        timedMove( 0.5,  0.5, 2000);  // diagonally forward-RIGHT
        timedMove(-0.5, -0.5, 2000);  // diagonally back to start
    }

    // Runs a mecanum mix (axial + lateral together) for a set time
    private void timedMove(double axial, double lateral, long milliseconds) {
        setMecanum(axial, lateral, 0);
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < milliseconds) {
            idle();
        }
        stopWheels();
    }

    // ---------------- routine 1: slide until touch, return ----------------

    private void slideUntilTouchRoutine() {
        // Measure this trip from zero
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        // 1) Slide RIGHT until the touch sensor is pressed.
        //    The 5-second cap is a safety stop in case the sensor never hits.
        runtime.reset();
        while (opModeIsActive() && !touch.isPressed() && runtime.seconds() < 5) {
            setMecanum(0, SLIDE_POWER, 0);   // lateral + = slide right
            pinpoint.update();
            telemetry.addData("sliding right", "Y = %.1f in",
                    pinpoint.getPosition().getY(DistanceUnit.INCH));
            telemetry.update();
        }
        stopWheels();

        double y = pinpoint.getPosition().getY(DistanceUnit.INCH);
        telemetry.addData("bumped wall at", "%.1f in sideways", y);
        telemetry.update();
        sleep(500);

        // 2) Slide back the opposite way until Y returns to ~0
        //    (Y is negative after sliding right, so we slide left).
        runtime.reset();
        while (opModeIsActive() && Math.abs(y) > 0.25 && runtime.seconds() < 5) {
            setMecanum(0, SLIDE_POWER * Math.signum(y), 0);
            pinpoint.update();
            y = pinpoint.getPosition().getY(DistanceUnit.INCH);
            telemetry.addData("returning", "Y = %.1f in", y);
            telemetry.update();
        }
        stopWheels();

        telemetry.addData("done", "back at start (Y = %.1f in)", y);
        telemetry.update();
    }

    // ---------------- movement helpers ----------------

    /*
     * Mecanum mixing — the sign-pattern table from the cheat sheet:
     *   axial   + = forward
     *   lateral + = slide right
     *   yaw     + = spin clockwise
     */
    private void setMecanum(double axial, double lateral, double yaw) {
        frontLeftDrive.setPower(axial + lateral + yaw);
        backLeftDrive.setPower(axial - lateral + yaw);
        frontRightDrive.setPower(axial - lateral - yaw);
        backRightDrive.setPower(axial + lateral - yaw);
    }

    // Runs LEFT side at leftPower and RIGHT side at rightPower for a set time
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

    private void stopWheels() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
