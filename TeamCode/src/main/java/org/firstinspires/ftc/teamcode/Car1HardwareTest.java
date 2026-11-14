package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Car 1 hardware test (11-04; config names follow the wiring table: lf lb rf rb / flywheelNectar flywheelPollen intake /
 * sorterGateL sorterGateR sorterEndL sorterEndR / pp). Only one device moves at a time.
 *
 * gamepad1:
 *   dpad up / down   pick the next device (the motor you leave stops by itself)
 *   B                stop everything (all motor power 0, flywheels off)
 *   4 drive motors / intake : left stick up / down = power (let go and it stops)
 *   2 flywheels      : X on / off (starts at 0.2 power), Y +0.1, A -0.1
 *   4 servos         : moves to 0.5 the first time you pick it; right bumper +0.02, left bumper -0.02;
 *                      Y = save the current position as "open", A = save it as "closed"
 *                      (written to the Hub file /sdcard/FIRST/car1_servo.txt; Car1TeleOp reads it at start)
 *   back             reset Pinpoint position and IMU (robot must be still)
 *
 * Driver Station shows: the selected device and its power / position, every motor's encoder count and speed
 * (ticks per second), and pp x / y (mm) and angle (degrees).
 * If a device is not set up (wrong name / not plugged in) the program does not crash; the screen just shows "not found".
 */
@TeleOp(name = "Car1 Hardware Test", group = "Test")
public class Car1HardwareTest extends LinearOpMode {

    private static final String[] MOTORS = {"lf", "lb", "rf", "rb", "flywheelNectar", "flywheelPollen", "intake"};
    private static final String[] SERVOS = {"sorterGateL", "sorterGateR", "sorterEndL", "sorterEndR"};
    private static final int N_MOTORS = MOTORS.length;
    private static final int N_DEVICES = MOTORS.length + SERVOS.length;

    private final DcMotorEx[] motor = new DcMotorEx[N_MOTORS];
    private final Servo[] servo = new Servo[SERVOS.length];
    private final double[] servoPos = new double[SERVOS.length];
    private final boolean[] servoTouched = new boolean[SERVOS.length];
    private final double[] flyPower = {0.2, 0.2};
    private final boolean[] flyOn = {false, false};
    private GoBildaPinpointDriver pp;

    private int sel = 0;
    private boolean pUp, pDown, pX, pY, pA, pB, pLB, pRB, pBack;

    @Override
    public void runOpMode() {
        for (int i = 0; i < N_MOTORS; i++) {
            try {
                motor[i] = hardwareMap.get(DcMotorEx.class, MOTORS[i]);
                motor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor[i].setZeroPowerBehavior(i < 4 ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
            } catch (Exception e) {
                motor[i] = null;
            }
        }
        // Wiring table section 5: the two left drive motors start as REVERSE (we guess the two sides are mirror images).
        // If a wheel spins the wrong way, change it here, don't swap wires
        setDir("lf", DcMotorSimple.Direction.REVERSE);
        setDir("lb", DcMotorSimple.Direction.REVERSE);
        setDir("intake", DcMotorSimple.Direction.REVERSE);   // 11-11 tested on robot: intake was backwards (same as Car1TeleOp)
        setDir("flywheelNectar", DcMotorSimple.Direction.REVERSE);   // 11-11 tested on robot: NECTAR flywheel was backwards

        for (int i = 0; i < SERVOS.length; i++) {
            try {
                servo[i] = hardwareMap.get(Servo.class, SERVOS[i]);
            } catch (Exception e) {
                servo[i] = null;
            }
            servoPos[i] = 0.5;
        }

        try {
            pp = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
            // 11-03: 4-bar odometry pods, X pod 15 mm left of center, Y pod 55 mm in front of center,
            // X / Y both REVERSED (tested on the old chassis; on 11-12 checked again on the new chassis with Car1 PP Test: still correct)
            pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                    GoBildaPinpointDriver.EncoderDirection.REVERSED);
            pp.setOffsets(15, 55, DistanceUnit.MM);   // if this line gives a compile error, delete ", DistanceUnit.MM" (old drivers only take mm)
            pp.resetPosAndIMU();
        } catch (Exception e) {
            pp = null;
        }

        telemetry.addLine("Car1 Hardware Test ready. Press PLAY to start. dpad up/down = next device, B = stop all");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            if (up && !pUp) changeSel(-1);
            if (down && !pDown) changeSel(+1);

            if (gamepad1.b && !pB) stopAll();

            if (sel < N_MOTORS) {
                if (sel == 4 || sel == 5) flywheel(sel - 4);
                else if (motor[sel] != null) motor[sel].setPower(-gamepad1.left_stick_y);
            } else {
                int s = sel - N_MOTORS;
                if (servo[s] != null) {
                    if (!servoTouched[s]) { servoTouched[s] = true; servo[s].setPosition(servoPos[s]); }
                    if (gamepad1.right_bumper && !pRB) servoPos[s] = clip(servoPos[s] + 0.02, 0, 1);
                    if (gamepad1.left_bumper && !pLB) servoPos[s] = clip(servoPos[s] - 0.02, 0, 1);
                    servo[s].setPosition(servoPos[s]);
                    if (gamepad1.y && !pY) ServoStore.save(SERVOS[s], true, servoPos[s]);
                    if (gamepad1.a && !pA) ServoStore.save(SERVOS[s], false, servoPos[s]);
                }
            }

            if (pp != null) {
                if (gamepad1.back && !pBack) pp.resetPosAndIMU();
                pp.update();
            }

            pUp = up; pDown = down; pX = gamepad1.x; pY = gamepad1.y; pA = gamepad1.a; pB = gamepad1.b;
            pLB = gamepad1.left_bumper; pRB = gamepad1.right_bumper; pBack = gamepad1.back;
            show();
        }
        stopAll();
    }

    private void flywheel(int k) {
        DcMotorEx m = motor[4 + k];
        if (gamepad1.x && !pX) { flyOn[k] = !flyOn[k]; if (flyOn[k]) flyPower[k] = 0.2; }
        if (gamepad1.y && !pY) flyPower[k] = clip(flyPower[k] + 0.1, 0, 1);
        if (gamepad1.a && !pA) flyPower[k] = clip(flyPower[k] - 0.1, 0, 1);
        if (m != null) m.setPower(flyOn[k] ? flyPower[k] : 0);
    }

    private void changeSel(int d) {
        // The device you leave: stick-controlled motors stop (flywheels keep their state, so you can watch the numbers
        // while switching to another device; press B to stop them)
        if (sel < N_MOTORS && sel != 4 && sel != 5 && motor[sel] != null) motor[sel].setPower(0);
        sel = (sel + d + N_DEVICES) % N_DEVICES;
    }

    private void stopAll() {
        for (DcMotorEx m : motor) if (m != null) m.setPower(0);
        flyOn[0] = false; flyOn[1] = false;
    }

    private void setDir(String name, DcMotorSimple.Direction dir) {
        for (int i = 0; i < N_MOTORS; i++) if (MOTORS[i].equals(name) && motor[i] != null) motor[i].setDirection(dir);
    }

    private static double clip(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    private void show() {
        String name = sel < N_MOTORS ? MOTORS[sel] : SERVOS[sel - N_MOTORS];
        telemetry.addData("> Selected", "%s  (%d/%d)", name, sel + 1, N_DEVICES);
        if (sel < N_MOTORS) {
            DcMotorEx m = motor[sel];
            if (m == null) telemetry.addLine("   Motor not found: check config name / wiring");
            else if (sel == 4 || sel == 5) telemetry.addData("   Flywheel", "%s  power %.1f  (X on/off | Y +0.1 | A -0.1)", flyOn[sel - 4] ? "on" : "off", flyPower[sel - 4]);
            else telemetry.addData("   Power", "%.2f  (left stick up/down)", m.getPower());
        } else {
            int s = sel - N_MOTORS;
            if (servo[s] == null) telemetry.addLine("   Servo not found: check config name / wiring");
            else {
                telemetry.addData("   Servo position", "%.2f  (right bumper +0.02 | left bumper -0.02)", servoPos[s]);
                telemetry.addData("   Saved", "open %s | closed %s  (Y save open | A save closed)", ServoStore.fmt(SERVOS[s], true), ServoStore.fmt(SERVOS[s], false));
                if (ServoStore.lastError != null) telemetry.addData("   Error saving file", ServoStore.lastError);
            }
        }
        telemetry.addLine("-- Motors: encoder / speed (ticks per second) / power");
        for (int i = 0; i < N_MOTORS; i++) {
            if (motor[i] == null) telemetry.addData(MOTORS[i], "not found");
            else telemetry.addData(MOTORS[i], "%d  /  %.0f  /  %.2f", motor[i].getCurrentPosition(), motor[i].getVelocity(), motor[i].getPower());
        }
        telemetry.addLine("-- Servos (only shows ones that have moved)");
        for (int i = 0; i < SERVOS.length; i++)
            telemetry.addData(SERVOS[i], servo[i] == null ? "not found" : (servoTouched[i] ? String.format("%.2f", servoPos[i]) : "not moved yet"));
        if (pp == null) telemetry.addData("pp", "not found (Pinpoint config name pp, Control Hub I2C bus 0)");
        else {
            Pose2D p = pp.getPosition();
            telemetry.addData("pp", "x %.1f mm   y %.1f mm   angle %.1f deg   (back to reset)",
                    p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM), p.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pp status", pp.getDeviceStatus());
        }
        telemetry.addLine("B stop all | dpad up/down next device");
        telemetry.update();
    }
}
