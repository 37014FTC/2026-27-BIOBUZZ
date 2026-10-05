package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.tuning.autotune.DisplayName;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;
import java.util.Properties;
import java.util.function.Function;

/*
 * The stock Foresight Tuner runs every sub-step in one go and loses all
 * results if you stop it early (the braking steps return an empty list,
 * and .get(0) then crashes the whole procedure).
 *
 * This Procedure runs ONE step per invocation and saves each result to
 * a file on the Control Hub (/sdcard/FIRST/pedro_foresight.properties),
 * so progress survives between runs, power cycles, and early stops.
 * Defaults are smaller than stock to fit tighter spaces.
 *
 * Run steps in order 1..10, then "Show / generate config" prints the
 * complete ForesightConfig block for Constants.java.
 *
 * Space needed per step (rough): 1/2: 24 in + drift. 3/4: short run +
 * coast. 5: room to turn back and forth. 6: spin in place. 7: forward
 * AND backward runway (5 trials each direction). 8: LATERAL runway both
 * ways (5 trials). 9: 12-24 in forward. 10: 12-24 in sideways.
 */
public class ForesightSteps extends Procedure {

    static final String RESULTS_FILE = "/sdcard/FIRST/pedro_foresight.properties";

    static final String[] ALL_KEYS = {
            "forwardVelocity", "strafeVelocity",
            "forwardDeceleration", "strafeDeceleration",
            "headingLinear", "headingQuadratic", "headingKP",
            "forwardLinear", "forwardQuadratic", "strafeLinear", "strafeQuadratic",
            "forwardTranslationalPrimary", "forwardTranslationalSecondary",
            "strafeTranslationalPrimary", "strafeTranslationalSecondary",
            "coast", "brake"
    };

    enum Step {
        @DisplayName("0. Heading kP (bounce-proof, saves)") HEADING_DEBUG,
        @DisplayName("1. Max Forward Velocity") FORWARD_VELOCITY,
        @DisplayName("2. Max Strafe Velocity") STRAFE_VELOCITY,
        @DisplayName("3. Forward Deceleration") FORWARD_DECELERATION,
        @DisplayName("4. Strafe Deceleration") STRAFE_DECELERATION,
        @DisplayName("5. Heading Braking") HEADING_BRAKING,
        @DisplayName("6. Heading kP") HEADING_TUNER,
        @DisplayName("7. Forward Braking (needs 5+6)") FORWARD_BRAKING,
        @DisplayName("8. Strafe Braking (needs 5+6)") STRAFE_BRAKING,
        @DisplayName("9. Forward Translational") FORWARD_TRANSLATIONAL,
        @DisplayName("10. Strafe Translational") STRAFE_TRANSLATIONAL,
        @DisplayName("Show / generate config") SHOW_CONFIG
    }

    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    public ForesightSteps(Function<HardwareMap, Localizer> localizerFunction,
                          Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Foresight Steps",
                "Runs ONE Foresight tuning step per run and saves results to "
                        + RESULTS_FILE + " so progress is never lost.");
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    public void run() throws InterruptedException {
        Properties props = loadResults();

        Inputs stepInputs = inputs("Foresight step",
                "Pick one step. Results accumulate on the hub between runs.");
        Inputs.Field<Step> step = stepInputs.e("Step", Step.class).withDefault(Step.FORWARD_VELOCITY);
        awaitInputs(stepInputs);

        switch (step.get()) {
            case HEADING_DEBUG: {
                double[] s = runOpMode(new HeadingDebug(localizerFunction, drivetrainFunction));
                if (s == null) { abort("Spin stopped early — re-run."); return; }
                double maxOmega = s[0], steady = s[1], tau = s[2], kP = s[3];
                int samples = (int) s[4];

                result("maxOmega (rad/s)", maxOmega);
                result("steady omega (rad/s)", steady);
                result("K = steady/0.4", steady / 0.4);
                result("tau (s, time to 63%)", tau);
                result("computed headingKP", kP);
                result("samples", samples);

                if (tau <= 0.02 || tau > 3.0) {
                    abort("tau = " + tau + " s looks wrong (need 0.02-3.0). "
                            + "Make sure the robot is free to spin and the floor is firm, then re-run.");
                    return;
                }
                if (kP < 0.05 || kP > 100.0) {
                    abort("computed kP = " + kP + " is outside the sane range 0.05-100 — re-run on a firmer floor.");
                    return;
                }
                // Foresight's heading loop error is in DEGREES, while the
                // tuner formula produces per-RADIAN gain — divide by 57.2958.
                // Saving the loop-ready value keeps future generated
                // configs from resurrecting the heading seizure.
                saveResult(props, "headingKP", kP / (180.0 / Math.PI));
                break;
            }
            case FORWARD_VELOCITY: {
                double d = askDistance(24.0);
                Double v = runOpMode(new ForwardVelocity(localizerFunction, drivetrainFunction, d));
                if (bad(v)) { abort("Step stopped early — re-run this step."); return; }
                saveResult(props, "forwardVelocity", v);
                break;
            }
            case STRAFE_VELOCITY: {
                double d = askDistance(24.0);
                Double v = runOpMode(new StrafeVelocity(localizerFunction, drivetrainFunction, d));
                if (bad(v)) { abort("Step stopped early — re-run this step."); return; }
                saveResult(props, "strafeVelocity", v);
                break;
            }
            case FORWARD_DECELERATION: {
                double vel = askVelocity(20.0);
                Double v = runOpMode(new ForwardDeceleration(localizerFunction, drivetrainFunction, vel));
                if (bad(v)) { abort("Step stopped early — re-run this step."); return; }
                saveResult(props, "forwardDeceleration", v);
                break;
            }
            case STRAFE_DECELERATION: {
                double vel = askVelocity(20.0);
                Double v = runOpMode(new StrafeDeceleration(localizerFunction, drivetrainFunction, vel));
                if (bad(v)) { abort("Step stopped early — re-run this step."); return; }
                saveResult(props, "strafeDeceleration", v);
                break;
            }
            case HEADING_BRAKING: {
                List<Double> r = runOpMode(new HeadingBraking(localizerFunction, drivetrainFunction));
                if (r == null || r.size() < 2) { abort("Step stopped early — re-run this step."); return; }
                saveResult(props, "headingLinear", r.get(0));
                saveResult(props, "headingQuadratic", r.get(1));
                break;
            }
            case HEADING_TUNER: {
                Double v = runOpMode(new HeadingTuner(localizerFunction, drivetrainFunction));
                if (bad(v)) { abort("Step stopped early — re-run this step."); return; }
                saveResult(props, "headingKP", v);
                break;
            }
            case FORWARD_BRAKING: {
                Double hl = require(props, "headingLinear", "step 5 (Heading Braking)");
                if (hl == null) return;
                Double hq = require(props, "headingQuadratic", "step 5 (Heading Braking)");
                if (hq == null) return;
                Double kp = require(props, "headingKP", "step 6 (Heading kP)");
                if (kp == null) return;
                double d = Math.max(askDistance(15.0), 15.0);
                List<Double> r = runOpMode(new ForwardBraking(localizerFunction, drivetrainFunction, hl, hq, kp, d));
                if (r == null || r.size() < 2) { abort("Step stopped early — re-run this step."); return; }
                saveResult(props, "forwardLinear", r.get(0));
                saveResult(props, "forwardQuadratic", r.get(1));
                break;
            }
            case STRAFE_BRAKING: {
                Double hl = require(props, "headingLinear", "step 5 (Heading Braking)");
                if (hl == null) return;
                Double hq = require(props, "headingQuadratic", "step 5 (Heading Braking)");
                if (hq == null) return;
                Double kp = require(props, "headingKP", "step 6 (Heading kP)");
                if (kp == null) return;
                double d = Math.max(askDistance(15.0), 15.0);
                List<Double> r = runOpMode(new StrafeBraking(localizerFunction, drivetrainFunction, hl, hq, kp, d));
                if (r == null || r.size() < 2) { abort("Step stopped early — re-run this step."); return; }
                saveResult(props, "strafeLinear", r.get(0));
                saveResult(props, "strafeQuadratic", r.get(1));
                break;
            }
            case FORWARD_TRANSLATIONAL: {
                List<Double> r = runOpMode(new ForwardTranslational(localizerFunction, drivetrainFunction));
                if (r == null || r.size() < 4) { abort("Step stopped early — re-run this step."); return; }
                saveResult(props, "forwardTranslationalPrimary", r.get(0));
                saveResult(props, "forwardTranslationalSecondary", r.get(1));
                saveResult(props, "coast", r.get(2));
                saveResult(props, "brake", r.get(3));
                break;
            }
            case STRAFE_TRANSLATIONAL: {
                List<Double> r = runOpMode(new StrafeTranslational(localizerFunction, drivetrainFunction));
                if (r == null || r.size() < 2) { abort("Step stopped early — re-run this step."); return; }
                saveResult(props, "strafeTranslationalPrimary", r.get(0));
                saveResult(props, "strafeTranslationalSecondary", r.get(1));
                break;
            }
            case SHOW_CONFIG: {
                for (String key : ALL_KEYS) {
                    if (props.getProperty(key) == null) {
                        abort("Missing: " + key + " — run its step first. Saved so far: "
                                + props.size() + " of " + ALL_KEYS.length + " values.");
                        return;
                    }
                }
                double ftp = get(props, "forwardTranslationalPrimary");
                double fts = get(props, "forwardTranslationalSecondary");
                double stp = get(props, "strafeTranslationalPrimary");
                double sts = get(props, "strafeTranslationalSecondary");
                double coast = get(props, "coast");
                double brake = get(props, "brake");
                double headingKP = get(props, "headingKP");
                double hl = get(props, "headingLinear");
                double hq = get(props, "headingQuadratic");
                double fl = get(props, "forwardLinear");
                double fq = get(props, "forwardQuadratic");
                double sl = get(props, "strafeLinear");
                double sq = get(props, "strafeQuadratic");
                double fwdVel = get(props, "forwardVelocity");
                double strVel = get(props, "strafeVelocity");
                double fwdDec = get(props, "forwardDeceleration");
                double strDec = get(props, "strafeDeceleration");

                code(Language.JAVA,
                        "public static ForesightConfig foresightConfig = new ForesightConfig(\n" +
                        "        c -> {\n" +
                        "            Controller primaryTranslationalForward = Controller.proportional(" + ftp + ");\n" +
                        "            Controller secondaryTranslationalForward = Controller.proportional(" + fts + ");\n" +
                        "            Controller primaryTranslationalLateral = Controller.proportional(" + stp + ");\n" +
                        "            Controller secondaryTranslationalLateral = Controller.proportional(" + sts + ");\n" +
                        "\n" +
                        "            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));\n" +
                        "            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));\n" +
                        "\n" +
                        "            c.coast.set(Controller.proportionalFeedforward(" + coast + "));\n" +
                        "            c.brake.set(Controller.proportionalFeedforward(" + brake + "));\n" +
                        "\n" +
                        "            c.headingFeedback.set(Controller.proportional(" + headingKP + "));\n" +
                        "            c.headingBrakeCoefficients.set(Vector2D.cartesian(" + hl + ", " + hq + "));\n" +
                        "\n" +
                        "            c.linearBrakeCoefficients.set(Matrix.diag(" + fl + ", " + sl + "));\n" +
                        "            c.quadraticBrakeCoefficients.set(Matrix.diag(" + fq + ", " + sq + "));\n" +
                        "\n" +
                        "            c.maxAchievableForwardVelocity.set(" + fwdVel + ");\n" +
                        "            c.maxAchievableStrafeVelocity.set(" + strVel + ");\n" +
                        "            c.naturalForwardDeceleration.set(" + fwdDec + ");\n" +
                        "            c.naturalStrafeDeceleration.set(" + strDec + ");\n" +
                        "        }\n" +
                        ");");
                break;
            }
        }

        // The web UI renders the Values card alongside code emission, so
        // emit the saved state after every step — this doubles as a
        // visible progress report of all collected values so far.
        if (step.get() != Step.SHOW_CONFIG) {
            StringBuilder summary = new StringBuilder();
            summary.append("// Foresight results saved so far: ")
                    .append(props.size()).append(" of ").append(ALL_KEYS.length).append("\n");
            for (String key : ALL_KEYS) {
                String v = props.getProperty(key);
                summary.append("// ").append(key).append(" = ")
                        .append(v != null ? v : "(not run yet)").append("\n");
            }
            code(Language.JAVA, summary.toString());
        }
    }

    // ---------------- helpers ----------------

    private double askDistance(double defaultValue) throws InterruptedException {
        Inputs i = inputs("Distance", "Inches. Smaller = less room needed, but very small hurts accuracy.");
        Inputs.Field<Double> d = i.d("Distance").withDefault(defaultValue);
        awaitInputs(i);
        return d.get();
    }

    private double askVelocity(double defaultValue) throws InterruptedException {
        Inputs i = inputs("Velocity", "Inches per second to reach before cutting power.");
        Inputs.Field<Double> v = i.d("Velocity").withDefault(defaultValue);
        awaitInputs(i);
        return v.get();
    }

    private boolean bad(Double v) {
        return v == null || Double.isNaN(v) || Double.isInfinite(v);
    }

    private Properties loadResults() {
        Properties p = new Properties();
        try (FileInputStream in = new FileInputStream(RESULTS_FILE)) {
            p.load(in);
        } catch (IOException ignored) {
            // first run — no file yet, start empty
        }
        return p;
    }

    private void saveResult(Properties p, String key, double value) throws InterruptedException {
        p.setProperty(key, String.valueOf(value));
        try (FileOutputStream out = new FileOutputStream(RESULTS_FILE)) {
            p.store(out, "Pedro Foresight step results (one line per completed step)");
        } catch (IOException e) {
            abort("Could not save to " + RESULTS_FILE + ": " + e);
        }
        result(key, value);
    }

    private Double require(Properties p, String key, String neededFor) throws InterruptedException {
        String v = p.getProperty(key);
        if (v == null) {
            abort("Missing " + key + " — run " + neededFor + " first.");
            return null;
        }
        return Double.parseDouble(v);
    }

    private double get(Properties p, String key) {
        return Double.parseDouble(p.getProperty(key));
    }
}

/*
 * A bounce-proof replacement for the stock Heading Tuner.
 *
 * The stock tuner log-fits the spin-up curve, which explodes (random-sign
 * huge kP) when omega vibrates — common on light mecanum chassis. This
 * version smooths omega with a moving average, reads the steady speed
 * directly, and measures tau as the time to reach 63% of steady state
 * (the textbook first-order definition). kP = tau * ALPHA^2 / K, same
 * formula the stock tuner uses.
 */
class HeadingDebug extends TuningOpMode<double[]> {
    static final double POWER = 0.4;
    static final double RUNTIME = 1.2;
    static final double ALPHA = 18.25;   // same constant as the stock Heading Tuner
    static final int SMOOTH_WINDOW = 10;

    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    public HeadingDebug(Function<HardwareMap, Localizer> localizerFunction,
                        Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Heading kP (bounce-proof)",
                "Spins at 0.4 power for 1.2 s and computes heading kP with "
                        + "vibration-tolerant math, then saves it. "
                        + "Robot must be on the floor and free to spin.",
                false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    protected double[] runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(com.pedropathing.math.Pose.zero());
        localizer.update();

        Thread.sleep(1000);
        waitForStart();

        localizer.setPose(com.pedropathing.math.Pose.zero());
        localizer.update();

        com.qualcomm.robotcore.util.ElapsedTime timer = new com.qualcomm.robotcore.util.ElapsedTime();
        java.util.List<Double> times = new java.util.ArrayList<>();
        java.util.List<Double> omegas = new java.util.ArrayList<>();

        drivetrain.drive(new com.pedropathing.drivetrain.DrivePowers(0, 0, POWER), false);
        while (timer.seconds() < RUNTIME && !isStopRequested()) {
            localizer.update();
            times.add(timer.seconds());
            omegas.add(Math.abs(localizer.velocity().omega));
        }
        drivetrain.stop();

        int n = omegas.size();
        if (n < 30) return null;   // not enough data

        double max = 0;
        for (double w : omegas) max = Math.max(max, w);

        // Moving-average smoothing to kill chassis vibration
        java.util.List<Double> smooth = new java.util.ArrayList<>();
        for (int i = 0; i < n; i++) {
            int from = Math.max(0, i - SMOOTH_WINDOW + 1);
            double sum = 0;
            for (int j = from; j <= i; j++) sum += omegas.get(j);
            smooth.add(sum / (i - from + 1));
        }

        // Steady state = average of the last 15 smoothed samples
        double steadySum = 0;
        int steadyCount = 0;
        for (int i = Math.max(0, n - 15); i < n; i++) { steadySum += smooth.get(i); steadyCount++; }
        double steady = steadySum / steadyCount;

        // tau = first time the smoothed curve reaches 63% of steady state
        double tau = -1;
        double target = 0.632 * steady;
        for (int i = 0; i < n; i++) {
            if (smooth.get(i) >= target) { tau = times.get(i); break; }
        }
        if (tau < 0) tau = RUNTIME;   // never reached 63%: bounded fallback

        double K = steady / POWER;
        double kP = (K > 0.01) ? tau * ALPHA * ALPHA / K : -1;

        return new double[]{max, steady, tau, kP, n};
    }
}
