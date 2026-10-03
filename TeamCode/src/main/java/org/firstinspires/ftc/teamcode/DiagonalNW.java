package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;

/*
 * 1 foot diagonally northwest while rotating 90 degrees clockwise —
 * the Pedro Pathing version. The entire maneuver is one path:
 * a straight line to the NW point with LINEAR heading interpolation
 * sweeping 0 -> -PI/2 (Pedro headings are RADIANS; -PI/2 = -90 deg,
 * negative = clockwise given the Pinpoint's heading convention).
 *
 * The Follower (Pinpoint localizer + mecanum drivetrain + Foresight
 * algorithm) handles all the mixing and correction internally —
 * see pedro/Constants.java for the hardware configuration.
 */
@Autonomous(name="Kingsley_DiagonalNW")
public class DiagonalNW extends LinearOpMode {

    static final double TARGET_DISTANCE_IN = 12;
    static final double TIMEOUT_S = 10;  // safety cap while gains are placeholders
    private static final double SQRT2 = Math.sqrt(2);
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());

        // The whole maneuver:
        Path diagonalSpin = line(
                Pose.zero(),
                new Pose(TARGET_DISTANCE_IN / SQRT2, TARGET_DISTANCE_IN / SQRT2, 0)
        ).linear(0, -Math.PI / 2);

        telemetry.addData(">", "Press START: 1 ft NW + 90 clockwise (Pedro).");
        telemetry.update();
        waitForStart();

        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(diagonalSpin);

        runtime.reset();
        while (opModeIsActive() && !follower.atParametricEnd() && runtime.seconds() < TIMEOUT_S) {
            follower.update();
            telemetry.addData("pose", follower.pose());
            telemetry.update();
        }

        // Hold the final pose so the robot doesn't drift away afterward
        follower.hold(new Pose(
                TARGET_DISTANCE_IN / SQRT2, TARGET_DISTANCE_IN / SQRT2, -Math.PI / 2));
        while (opModeIsActive()) {
            follower.update();
        }
    }
}
