package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Standalone copy of the AutoTune "Odometry Test" — same three phases,
 * same verdicts, but with NO framework in the way:
 *   - prints pose to DS telemetry the entire time (even mid-phase)
 *   - wraps everything in try/catch so a crash (e.g. flaky X pod cable)
 *     shows up as text instead of silently killing the opmode
 *
 * Phases: forward 1 s -> left 1 s -> turn 1 s (0.5 power), then verdicts:
 *   xPod / yPod / Heading — each Good or Flipped (+ resolution notes).
 * Reads verdicts on the Driver Station. Press STOP when done.
 */
@Autonomous(name="37014_OdometryCheck")
public class OdometryCheck extends LinearOpMode {

    static final double POWER = 0.5;
    static final String TAG = "37014OC";   // laptop log tag — readable via adb

    @Override
    public void runOpMode() {
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        Localizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);

        double totalHeading = 0;
        double prevHeading = 0;
        String verdictX = "?", verdictY = "?", verdictH = "?";
        Pose finalPose = Pose.zero();
        boolean verdictsLogged = false;

        telemetry.addData(">", "Press START. Robot self-drives 3 phases.");
        telemetry.update();
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        int phase = 0; // 0 fwd, 1 left, 2 turn, 3 done
        localizer.setPose(Pose.zero());
        localizer.update();
        prevHeading = 0;

        try {
            while (opModeIsActive()) {
                localizer.update();
                Pose pose = localizer.pose();
                finalPose = pose;

                // accumulate heading deltas (so wrap-around doesn't zero out)
                totalHeading += Angle.normalizeSigned(pose.heading() - prevHeading);
                prevHeading = pose.heading();

                switch (phase) {
                    case 0:
                        drivetrain.drive(new DrivePowers(POWER, 0, 0), false);
                        if (timer.seconds() > 1) { phase = 1; timer.reset();
                            RobotLog.ii(TAG, "phase1 forward done: " + poseLog(pose)); }
                        break;
                    case 1:
                        drivetrain.drive(new DrivePowers(0, POWER, 0), false);
                        if (timer.seconds() > 1) { phase = 2; timer.reset();
                            RobotLog.ii(TAG, "phase2 left done: " + poseLog(pose)); }
                        break;
                    case 2:
                        drivetrain.drive(new DrivePowers(0, 0, POWER), false);
                        if (timer.seconds() > 1) { phase = 3; timer.reset();
                            RobotLog.ii(TAG, "phase3 turn done: " + poseLog(pose)
                                    + " totalHeading=" + Math.toDegrees(totalHeading) + "deg"); }
                        break;
                    case 3:
                        drivetrain.drive(new DrivePowers(0, 0, 0), true);

                        // ---- verdicts (same thresholds as the stock test) ----
                        if (pose.x() < 0)      verdictX = "FLIPPED (negative)";
                        else if (pose.x() < 2) verdictX = "resolution too high";
                        else if (pose.x() > 144) verdictX = "resolution too low";
                        else                   verdictX = "Good";

                        if (pose.y() < 0)      verdictY = "FLIPPED (negative)";
                        else if (pose.y() < 2) verdictY = "resolution too high";
                        else if (pose.y() > 144) verdictY = "resolution too low";
                        else                   verdictY = "Good";

                        if (totalHeading < 0)        verdictH = "FLIPPED (negative)";
                        else if (totalHeading < 0.02) verdictH = "resolution too high";
                        else if (totalHeading > 2 * Math.PI) verdictH = "resolution too low";
                        else                          verdictH = "Good";

                        if (!verdictsLogged) {
                            verdictsLogged = true;
                            RobotLog.ii(TAG, "VERDICT xPod=" + verdictX
                                    + " | yPod=" + verdictY
                                    + " | Heading=" + verdictH
                                    + " | final " + poseLog(pose)
                                    + " totalHeading=" + Math.toDegrees(totalHeading) + "deg");
                        }
                        break;
                }

                telemetry.addData("phase", phase == 3 ? "DONE — read verdicts"
                        : new String[]{"forward", "left", "turn"}[phase]);
                telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f deg",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.addData("totalHeading", "%.1f deg", Math.toDegrees(totalHeading));
                if (phase == 3) {
                    telemetry.addLine("--- VERDICTS ---");
                    telemetry.addData("xPod", verdictX);
                    telemetry.addData("yPod", verdictY);
                    telemetry.addData("Heading", verdictH);
                }
                telemetry.update();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG, "EXCEPTION: " + e);
            drivetrain.drive(new DrivePowers(0, 0, 0), true);
            telemetry.addLine("!!! EXCEPTION mid-test !!!");
            telemetry.addData("error", e.toString());
            telemetry.addData("last pose", "x=%.1f y=%.1f h=%.1f",
                    finalPose.x(), finalPose.y(), finalPose.heading());
            telemetry.update();
            sleep(10000); // keep it on screen long enough to read
        }
    }

    private String poseLog(Pose p) {
        return String.format("(x=%.1f y=%.1f h=%.1fdeg)",
                p.x(), p.y(), Math.toDegrees(p.heading()));
    }
}
