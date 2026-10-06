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
 * Frame probe — measures which physical direction the follower moves for
 * each coordinate axis. NO rotation, one axis at a time, gentle distances.
 *
 * Phase 1: drives along +X.   Watch: does the robot go FORWARD or BACKWARD?
 * Phase 2: drives along +Y.   Watch: does the robot go LEFT or RIGHT?
 *
 * Tell someone (me) those two answers and the whole coordinate system is
 * known — no more guessing.
 */
@Autonomous(name="37014_FrameProbe")
public class FrameProbe extends LinearOpMode {

    static final double PROBE_DISTANCE_IN = 24;
    static final double TIMEOUT_S = 6;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);

        Path xAxisProbe = line(Pose.zero(),
                new Pose(PROBE_DISTANCE_IN, 0, 0)).constant(0);
        Path yAxisProbe = line(Pose.zero(),
                new Pose(0, PROBE_DISTANCE_IN, 0)).constant(0);

        telemetry.addData(">", "Phase 1: +X. Phase 2: +Y. Watch directions!");
        telemetry.update();
        waitForStart();

        // ---- Phase 1: +X ----
        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(xAxisProbe);
        runtime.reset();
        telemetry.addData("phase", "1: driving +X");
        telemetry.update();
        while (opModeIsActive() && !follower.atParametricEnd()
                && runtime.seconds() < TIMEOUT_S) {
            follower.update();
        }
        stopFollower(follower);
        sleep(2000);

        // ---- Phase 2: +Y ----
        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(yAxisProbe);
        runtime.reset();
        telemetry.addData("phase", "2: driving +Y");
        telemetry.update();
        while (opModeIsActive() && !follower.atParametricEnd()
                && runtime.seconds() < TIMEOUT_S) {
            follower.update();
        }
        stopFollower(follower);

        // ---- Phase 3: tiny move + heading sweep to -90° ----
        // Watch: does the robot rotate RIGHT (clockwise, correct) or
        // LEFT (counterclockwise, mirrored)? And what's its final heading
        // on the telemetry?
        sleep(2000);
        Path spinProbe = line(Pose.zero(),
                new Pose(6, 0, 0)).linear(0, -Math.PI / 2);
        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(spinProbe);
        runtime.reset();
        telemetry.addData("phase", "3: sweep to -90 deg");
        telemetry.update();
        while (opModeIsActive() && !follower.atParametricEnd()
                && runtime.seconds() < TIMEOUT_S) {
            follower.update();
            telemetry.addData("heading", follower.pose().heading());
            telemetry.update();
        }
        stopFollower(follower);

        telemetry.addData("done", "which way did phase 3 rotate?");
        telemetry.update();
    }

    private void stopFollower(Follower follower) {
        // park in place briefly so momentum settles, then hold
        follower.hold(follower.pose());
    }
}
