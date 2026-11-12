# Tuning Evidence — Verified Record of Foresight Parameter Identification

**Team 37014 · BIOBUZZ 2026–27 · Prepared for the engineering notebook**
**Values measured: 2026-10-04 to 2026-10-05 · This document: 2026-11-12**

This document proves, with independently verifiable sources, that all 17
motion-control parameters in our Foresight configuration were identified
empirically on this robot — not copied from defaults or another team.

---

## The evidence chain (three independent sources)

| # | Source | Where to verify | What it proves |
|---|---|---|---|
| 1 | **Git commit `a65ae19`** — dated **2026-10-05 16:25 +1100** | [github.com/37014FTC/2026-27-BIOBUZZ/commit/a65ae19](https://github.com/37014FTC/2026-27-BIOBUZZ/commit/a65ae19) | The measured values entering the codebase, two days after the wiring commit. The diff shows placeholder values being *replaced* by measurements. |
| 2 | **Preceding commit `d0bddc4`** — dated **2026-10-03** | [github.com/37014FTC/2026-27-BIOBUZZ/commit/d0bddc4](https://github.com/37014FTC/2026-27-BIOBUZZ/commit/d0bddc4) | The *before* state: hardware wired, config empty/placeholder. Together with #1 it brackets the tuning work in a two-day window. |
| 3 | **On-robot results file** — `/sdcard/FIRST/pedro_foresight.properties` on the Control Hub | Pull via USB: see instructions below | The tuner's own persisted measurements, written by the robot at measurement time. Values must match #1. |

Git commits are content-addressed (SHA hashes): the values cannot be
altered after the fact without changing the hash and every subsequent
commit. The full history is public.

---

## The measured values (as committed in `a65ae19`, 2026-10-05)

| Parameter | Measured value | How it was measured |
|---|---|---|
| maxAchievableForwardVelocity | **80.93 in/s** | Full-power 48-in sprint, velocity averaged over final samples |
| maxAchievableStrafeVelocity | **69.84 in/s** | Full-power 48-in lateral sprint |
| naturalForwardDeceleration | **761.5 in/s²** | Reach-velocity-then-coast identification |
| naturalStrafeDeceleration | **811.4 in/s²** | Lateral equivalent |
| forwardTranslational kP (primary/secondary) | **2.544 / 0.940** | System-identification from 12–24 in moves |
| strafeTranslational kP (primary/secondary) | **0.948 / 0.350** | Lateral equivalent |
| coast kV / brake kV | **0.01452 / 0.01234** | Feedforward fit from translational trials |
| headingBrakeCoefficients (linear/quadratic) | **0.0963 / −0.00277** | Turn-and-coast braking trials |
| forward linear/quadratic brake | **0.1218 / 3.96e-4** | 5-trial sprint-and-stop series, 15 in |
| strafe linear/quadratic brake | **0.01\* / 4.07e-3** | Lateral equivalent (\*see note) |
| headingFeedback kP | see note | Vibration-tolerant method — see note |

**Notes:**
- \* strafeLinear was fitted as −0.0504; a negative linear brake coefficient
  can produce an imaginary braking speed, so it was clamped to 0.01 —
  documented in the commit message of `a65ae19`.
- **headingFeedback**: the stock tuner's log-fit produced non-repeatable,
  sign-flipping results on our chassis (documented in PROJECT_LOG.md —
  chassis vibration makes omega bounce and the fit diverges). We built a
  replacement measurement (moving-average smoothing, tau from
  time-to-63%-of-steady-state) which yielded a repeatable ≈10.9 per radian;
  the deployed gain was then validated empirically by convergence testing
  (0.5) rather than direct use of the fitted value.

**Physics consistency checks (done at measurement time):**
- strafe/forward velocity ratio = **86%** — consistent with mecanum roller
  mechanics (strafe is always slower than forward)
- measured max spin rate ≈ **180°/s at 0.5 power** (independently confirmed
  by stopwatch: 360° in 4 s at 0.2 power) — cross-validates the rotational
  identification

---

## Methodology summary

Each parameter was identified with a dedicated single-purpose test using
our custom step-by-step tuner (ForesightSteps — itself in this repository,
`pedro/procedures/ForesightSteps.java`), which:

- runs **one identification test per invocation** (the stock tuner loses all
  results if interrupted — ours persists each step to the robot's storage)
- saves every result to `/sdcard/FIRST/pedro_foresight.properties` on the hub
- sanity-gates every measurement against physical plausibility ranges

Testing surface: hard floor (hallway), fresh battery. Full narrative of the
tuning sessions, including the failures and the root-cause investigation of
a direction-flipped odometry pod discovered during validation:
**[PROJECT_LOG.md](PROJECT_LOG.md)**.

---

## Verifying source #3 (the on-robot file)

With the robot connected to a laptop via USB and powered on:

```
adb pull /sdcard/FIRST/pedro_foresight.properties
```

The file lists each parameter and its measured value, written at
measurement time (Oct 4–5). Expected: values matching the table above.
The file's last-modified date on the hub reflects the final measurement
session.
