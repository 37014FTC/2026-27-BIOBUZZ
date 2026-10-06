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
 * Two-leg mission (frame: +X north, +Y west — verified by frame probe):
 *
 *   A (0,0) facing north
 *    -> B (8.5, 8.5): NW diagonal, 1 ft, rotating 90 deg CLOCKWISE in
 *       flight (sweep completes at 70% of the leg) — arrives facing EAST
 *    -> C (8.5, 0): since B faces east and C is east of B, this leg is a
 *       plain FORWARD DRIVE ~8.5 in, holding east-facing. C lands exactly
 *       on the north axis through A.
 *
 * No rotation at A, B, or C: all rotation happens during leg 1 travel.
 * Final telemetry shows the pose — score is x~8.5, y~0, heading ~-90.
 */
@Autonomous(name="37014_ABC")
public class ABCPath extends LinearOpMode {

    static final double TARGET_DISTANCE_IN = 12;   // A->B diagonal length
    static final double SWEEP_DONE_AT = 0.7;       // fraction of leg 1
    static final double TIMEOUT_S = 8;             // per leg
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

        final double bx = TARGET_DISTANCE_IN / SQRT2;  // B and C north-offset
        final double by = TARGET_DISTANCE_IN / SQRT2;  // B west-offset

        Path leg1 = line(Pose.zero(), new Pose(bx, by, 0))
                .heading((path, t) -> -Math.PI / 2 * Math.min(t / SWEEP_DONE_AT, 1.0));
        Path leg2 = line(new Pose(bx, by, 0), new Pose(bx, 0, 0))
                .constant(-Math.PI / 2);   // hold east-facing, no rotation

        telemetry.addData(">", "A->B (NW + 90 in flight, face east) -> C (drive east)");
        telemetry.update();
        waitForStart();

        follower.setPose(Pose.zero());
        follower.update();

        // ---- leg 1: diagonal + 180 in flight ----
        follower.follow(leg1);
        runtime.reset();
        while (opModeIsActive() && !follower.atParametricEnd()
                && runtime.seconds() < TIMEOUT_S) {
            follower.update();
        }
        // no rotation at B: roll straight into leg 2
        follower.follow(leg2);
        runtime.reset();
        while (opModeIsActive() && !follower.atParametricEnd()
                && runtime.seconds() < TIMEOUT_S) {
            follower.update();
        }

        // no rotation at C: stop clean
        follower.stop();
        drivetrain.drive(new DrivePowers(0, 0, 0), true);
        while (opModeIsActive()) {
            Pose p = follower.pose();
            telemetry.addData("final pose", "x=%.1f y=%.1f h=%.1f deg",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("target", "x=%.1f y=0 h=-90", bx);
            telemetry.update();
            idle();
        }
    }
}
