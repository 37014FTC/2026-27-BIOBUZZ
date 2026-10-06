package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Arcade drive + holonomic left stick (the standard FTC mecanum layout):
 *   Left stick    -> move in ANY direction (Y = forward/back, X = strafe,
 *                    both together = diagonals)
 *   Right stick X -> turn left / right (analog)
 *   Hold A        -> slow mode (40%)
 */
@TeleOp(name="Kingsley_ArcadeTeleop")
public class ArcadeTeleop extends LinearOpMode {

    static final double SLOW_MODE_FACTOR = 0.4;
    static final double MAX_POWER = 1.0;   // full stick = full voltage

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

        telemetry.addData(">", "Arcade: L stick = drive, R stick = turn, A = slow.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double axial   = -gamepad1.left_stick_y;   // + = forward
            double lateral =  gamepad1.left_stick_x;   // + = slide right
            double yaw     =  gamepad1.right_stick_x;  // + = turn right (CW)

            if (gamepad1.a) {
                axial   *= SLOW_MODE_FACTOR;
                lateral *= SLOW_MODE_FACTOR;
                yaw     *= SLOW_MODE_FACTOR;
            }

            // Full mecanum mix — left stick translates (diagonals included),
            // right stick turns
            frontLeft.setPower(scale(axial + lateral + yaw));
            backLeft.setPower(scale(axial - lateral + yaw));
            frontRight.setPower(scale(axial - lateral - yaw));
            backRight.setPower(scale(axial + lateral - yaw));

            telemetry.addData("drive / strafe / turn", "%.2f / %.2f / %.2f%s",
                    axial, lateral, yaw, gamepad1.a ? "  SLOW" : "");
            telemetry.update();
        }
    }

    // Clamp to +-1 first, THEN scale — so combined inputs cap at MAX_POWER
    // instead of clipping back up to 100%.
    private double scale(double p) {
        return Math.max(-1.0, Math.min(1.0, p)) * MAX_POWER;
    }
}
