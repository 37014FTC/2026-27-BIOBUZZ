package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * FIELD-ABSOLUTE waypoint teleop (v0 of the macro system).
 *
 * Coordinate system (the "final verdict"):
 *   - (0, 0) = CENTER OF THE FIELD
 *   - Units are FEET (field is +-6 ft each way; tile lines = whole/half ft)
 *   - +X = north (away from driver wall at heading 0), +Y = WEST (left)
 *   - Heading 0 = north, CCW positive (matches Pinpoint)
 *
 * Controls:
 *   Left stick     -> manual field-centric driving (in FIELD directions)
 *   Right stick X  -> turn
 *   X (press)      -> AUTO: drive to WAYPOINT_1
 *   A (hold)       -> slow mode (manual only)
 *   LEFT BUMPER    -> DRIVER RE-ZERO: point the robot's nose away from
 *                     you, tap this, and stick-forward becomes "away
 *                     from me" — controls follow YOU wherever you stand.
 *                     (AUTO waypoints stay field-absolute — unaffected.)
 *   Touch a stick  -> INSTANTLY cancels AUTO (driver is always boss)
 *
 * Safety: any stick input overrides; 4 s timeout per macro; telemetry
 * always shows who is driving.
 *
 * SETUP: at INIT the robot is TOLD its starting coordinates (constants
 * below) — place the robot on its start mark facing north before INIT.
 */
@TeleOp(name="37014_FieldMacro")
public class FieldMacroTeleop extends LinearOpMode {

    // ---- field setup (FEET / DEGREES — edit these) ----
    static final double START_X_FT = 0.0;      // where the robot is placed
    static final double START_Y_FT = -5.5;     // 5.5 ft south of center
    static final double START_HEADING_DEG = 0; // facing north

    // ---- waypoint 1 (X button) — FEET / DEGREES ----
    static final double WAYPOINT1_X_FT = 0.0;  // field center
    static final double WAYPOINT1_Y_FT = 0.0;
    static final double WAYPOINT1_HEADING_DEG = 0;

    static final double SLOW_MODE_FACTOR = 0.4;
    static final double MAX_POWER = 1.0;

    // auto-move tuning (proven values from the hand-rolled follower era)
    static final double TRANS_kP = 0.05;    // power per inch remaining
    static final double TRANS_MIN = 0.08;
    static final double TRANS_MAX = 0.4;
    static final double HEAD_kP = 0.5;      // power per radian of heading error
    static final double YAW_CAP = 0.35;
    static final double ARRIVE_RADIUS_IN = 1.8;   // ~0.15 ft
    static final double AUTO_TIMEOUT_S = 4.0;
    static final double STICK_OVERRIDE = 0.15;

    private static final double IN_PER_FT = 12.0;

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private GoBildaPinpointDriver pinpoint;
    private final ElapsedTime runtime = new ElapsedTime();

    private boolean autoMode = false;
    private boolean xWasDown = false;
    private double targetX = 0, targetY = 0, targetH = 0; // inches/rad

    // Driver-frame offset (radians). 0 = stick-forward is field north.
    // Left bumper sets it to the current heading, so stick-forward
    // becomes "away from the driver" wherever they stand.
    private double driverOffset = 0;

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "lf");
        backLeft   = hardwareMap.get(DcMotor.class, "lb");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backRight  = hardwareMap.get(DcMotor.class, "rb");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        setBrakes();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        pinpoint.setOffsets(-50, -30, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();   // robot STILL — calibrates IMU

        // THE RE-BASE: declare our start pose in FIELD coordinates.
        // From here on every reading is field-absolute (center = 0,0).
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                START_X_FT * IN_PER_FT,
                START_Y_FT * IN_PER_FT,
                AngleUnit.DEGREES,
                START_HEADING_DEG));

        telemetry.addData(">", "Place on start mark facing north, then INIT done.");
        telemetry.addData("start (ft)", "(%.1f, %.1f) h=%.0f",
                START_X_FT, START_Y_FT, START_HEADING_DEG);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double px = pose.getX(DistanceUnit.INCH);
            double py = pose.getY(DistanceUnit.INCH);
            double h  = pose.getHeading(AngleUnit.RADIANS);

            // ---- trigger: X press starts the macro ----
            boolean xDown = gamepad1.x;
            if (xDown && !xWasDown && !autoMode) {
                targetX = WAYPOINT1_X_FT * IN_PER_FT;
                targetY = WAYPOINT1_Y_FT * IN_PER_FT;
                targetH = Math.toRadians(WAYPOINT1_HEADING_DEG);
                autoMode = true;
                runtime.reset();
            }
            xWasDown = xDown;

            // ---- driver re-zero: controls now relative to current nose ----
            if (gamepad1.left_bumper) {
                driverOffset = h;
            }

            // ---- safety: sticks override AUTO instantly ----
            boolean stickTouched =
                    Math.abs(gamepad1.left_stick_y) > STICK_OVERRIDE
                 || Math.abs(gamepad1.left_stick_x) > STICK_OVERRIDE
                 || Math.abs(gamepad1.right_stick_x) > STICK_OVERRIDE;
            if (autoMode && stickTouched) {
                cancelAuto();
            }

            if (autoMode) {
                driveToward(px, py, h);
                if (runtime.seconds() > AUTO_TIMEOUT_S) {
                    cancelAuto();   // safety: never grind into a wall
                }
            } else {
                manualFieldCentric(h);
            }

            telemetry.addData("mode", autoMode ? "AUTO -> waypoint" : "MANUAL");
            telemetry.addData("pose (ft)", "(%.2f, %.2f) h=%.0f deg",
                    px / IN_PER_FT, py / IN_PER_FT, Math.toDegrees(h));
            if (autoMode) {
                telemetry.addData("target (ft)", "(%.2f, %.2f)",
                        targetX / IN_PER_FT, targetY / IN_PER_FT);
                telemetry.addData("remaining", "%.1f in",
                        Math.hypot(targetX - px, targetY - py));
                telemetry.addData("timeout in", "%.1f s",
                        AUTO_TIMEOUT_S - runtime.seconds());
            }
            telemetry.update();
        }
    }

    // ---------------- MANUAL: field-centric driving ----------------

    private void manualFieldCentric(double h) {
        // rotate the driver's frame by the re-zero offset: stick-forward
        // means "h = driverOffset direction", not necessarily field north
        double hr = normalizeRad(h - driverOffset);

        double fieldForward = -gamepad1.left_stick_y;  // + = driver "north"
        double fieldEast    =  gamepad1.left_stick_x;  // + = driver "east"

        double axial   = fieldForward * Math.cos(hr) - fieldEast * Math.sin(hr);
        double lateral = fieldForward * Math.sin(hr) + fieldEast * Math.cos(hr);
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
    }

    // ---------------- AUTO: seek the waypoint ----------------

    private void driveToward(double px, double py, double h) {
        double ex = targetX - px;   // field-frame error (inches, +X north)
        double ey = targetY - py;   // +Y west
        double dist = Math.hypot(ex, ey);

        if (dist < ARRIVE_RADIUS_IN) {
            cancelAuto();   // arrived
            return;
        }

        // rotate field-frame error into the robot's body frame
        double axial   = ex * Math.cos(h) + ey * Math.sin(h);   // fwd
        double strafeL = -ex * Math.sin(h) + ey * Math.cos(h);  // leftward
        double lateral = -strafeL;                              // lateral+ = right

        // scale to a bounded power (proportional approach)
        double mag = Math.max(TRANS_MIN, Math.min(TRANS_MAX, TRANS_kP * dist));
        axial   = axial / dist * mag;
        lateral = lateral / dist * mag;

        // heading hold toward the waypoint's heading (CCW positive;
        // wheel yaw + = clockwise, hence the negation)
        double hErr = normalizeRad(targetH - h);
        double yaw = Math.max(-YAW_CAP, Math.min(YAW_CAP, HEAD_kP * hErr));

        frontLeft.setPower(scale(axial + lateral - yaw));
        backLeft.setPower(scale(axial - lateral - yaw));
        frontRight.setPower(scale(axial - lateral + yaw));
        backRight.setPower(scale(axial + lateral + yaw));
    }

    private void cancelAuto() {
        autoMode = false;
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    // ---------------- helpers ----------------

    private double normalizeRad(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private double scale(double p) {
        return Math.max(-1.0, Math.min(1.0, p)) * MAX_POWER;
    }

    private void setBrakes() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
