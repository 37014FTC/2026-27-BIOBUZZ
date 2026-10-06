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

/*
 * The finished DiagonalNW mission, iterated twice in one run:
 * each rep = 1-ft diagonal (robot-frame NW) while rotating 90 deg
 * clockwise in flight (sweep done at 70% of the path), arriving facing
 * the rep's starting-east.
 *
 * Rep 2 begins immediately where rep 1 ended — no stop, no rotation
 * between reps. The pose re-zeros at the start of each rep, so the
 * maneuver repeats relative to the robot's CURRENT facing:
 *   rep 1 (field frame): travel NW, end facing field-east
 *   rep 2 (local frame): travel its-NW = field NE, end facing field-south
 * Net path on the floor: a NW-then-NE zigzag.
 */
@Autonomous(name="37014_DiagonalNW_x2")
public class DiagonalTwice extends LinearOpMode {

    static final double TARGET_DISTANCE_IN = 12;
    static final double SWEEP_DONE_AT = 0.7;   // gentler ramp = less tracking lag
    static final double TIMEOUT_S = 8;   // per rep
    static final int REPS = 2;

    private static final double SQRT2 = Math.sqrt(2);
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Built from parts so we can explicitly zero the motors at the end
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        Follower follower = new Follower(
                new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                drivetrain,
                new Foresight(Constants.foresightConfig));

        Path diagonalSpin = line(Pose.zero(),
                new Pose(TARGET_DISTANCE_IN / SQRT2, TARGET_DISTANCE_IN / SQRT2, 0))
                .heading((path, t) -> -Math.PI / 2 * Math.min(t / SWEEP_DONE_AT, 1.0));

        telemetry.addData(">", "DiagonalNW mission x%d, continuous.", REPS);
        telemetry.update();
        waitForStart();

        for (int rep = 0; rep < REPS && opModeIsActive(); rep++) {
            // Each rep starts a fresh local frame wherever the robot is
            follower.setPose(Pose.zero());
            follower.update();
            follower.follow(diagonalSpin);

            runtime.reset();
            while (opModeIsActive() && !follower.atParametricEnd()
                    && runtime.seconds() < TIMEOUT_S) {
                follower.update();
                telemetry.addData("rep", "%d/%d  pose %s", rep + 1, REPS, follower.pose());
                telemetry.update();
            }
            // no pause — roll straight into the next rep
        }

        follower.stop();
        drivetrain.drive(new DrivePowers(0, 0, 0), true);
        while (opModeIsActive()) {
            telemetry.addData("done", "%d reps complete", REPS);
            telemetry.update();
            idle();
        }
    }
}
