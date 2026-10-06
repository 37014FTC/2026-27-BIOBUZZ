package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;
// claude --resume 91ec36b0-e1aa-4146-8fa7-5d2ba3a1f0c8
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
@Autonomous(name="37014_DiagonalNW")
public class DiagonalNW extends LinearOpMode {

    static final double TARGET_DISTANCE_IN = 12;
    static final double TIMEOUT_S = 10;  // safety cap while gains are placeholders
    private static final double SQRT2 = Math.sqrt(2);
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Built from parts (not Constants.create) so we keep a reference to
        // the drivetrain and can explicitly zero the motors at the end —
        // follower.stop() alone may leave the last wheel powers latched.
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        Follower follower = new Follower(
                new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                drivetrain,
                new Foresight(Constants.foresightConfig));
        follower.setPose(Pose.zero());

        // The whole maneuver. Frame probe: +X forward, +Y LEFT, so
        // northwest = positive Y. Heading target sweeps 0 -> -90 deg,
        // reaching EAST at 70% of the path and holding — so the rotation
        // finishes IN FLIGHT and no rotation happens at either endpoint
        // (win condition: start A facing north, finish B facing east,
        // no rotation at A or B).
        Path diagonalSpin = line(
                Pose.zero(),
                new Pose(TARGET_DISTANCE_IN / SQRT2, TARGET_DISTANCE_IN / SQRT2, 0)
        ).heading((path, t) -> -Math.PI / 2 * Math.min(t / 0.7, 1.0));

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

        // Win condition: no rotation at point B — so NO hold tail. The
        // rotation must already be done when the path ends. Stop clean.
        follower.stop();
        drivetrain.drive(new DrivePowers(0, 0, 0), true);  // motors OFF for real
        while (opModeIsActive()) {
            telemetry.addData("final pose", follower.pose());
            telemetry.addData("target heading", "-90 deg (east)");
            telemetry.update();
            idle();
        }
    }
}
