# 🤖 FRC-style documentation, FTC-sized team — Team 37014

**FIRST Tech Challenge · 2026–27 BIOBUZZ season · Australia**

We are a small software-driven FTC team — three year-8 students who
designed, built, programmed, tuned, and documented this robot ourselves,
with mentor support on strategy and logistics. This repository is our
robot's complete software system **and** our engineering record: every
program, every decision, and every mistake we made learning them.

---

## What's in this repository

| Area | Contents |
|---|---|
| **Driver controls** | Six teleop modes (tank, arcade, field-centric, waypoint-assisted), each with rookie/experienced power tiers and precision slow-mode |
| **Autonomous** | Path-following missions using Pedro Pathing with odometry-localized mecanum drive — including in-motion heading sweeps and chained maneuvers |
| **Waypoint macros** | A field-absolute coordinate system (field center = origin, feet units) with button-triggered autonomous repositioning during teleop and driver-frame re-zeroing |
| **Tuning infrastructure** | A custom step-by-step replacement for the stock auto-tuner, with persistent results, sanity gates, and a vibration-tolerant heading-gain measurement method |
| **Diagnostics** | Standalone motor, odometry, and coordinate-frame verification tools with laptop log streaming |
| **Documentation** | `TeamCode/PROJECT_LOG.md` — the full engineering log, failures included |

## Engineering highlights

- **Root-cause odometry investigation.** A multi-day behavioral fault (path mirroring, unstable heading control) was traced to a single direction-inverted odometry pod in the localization config. The diagnosis path — isolating motors, verifying sensor data live, building a standalone frame probe — is documented step by step in PROJECT_LOG.md.
- **Rebuilt tuning tooling when the stock tools failed.** The stock tuner lost all results on early exit and its heading fit diverged on chassis vibration; we wrote a replacement that persists results per-step to the robot, validates outputs against physical sanity ranges, and measures heading response with a method robust to signal noise.
- **Safety-first control design.** All driver-assist autonomy features obey three rules: any manual input instantly overrides, every autonomous action is time-bounded, and mode state is always visible to the driver.
- **Measured, not guessed.** All 17 motion-control parameters were identified empirically on the robot (velocities, braking profiles, rotational response) and validated against physics consistency checks.

## For judges & visitors

- **[`TeamCode/PROJECT_LOG.md`](TeamCode/PROJECT_LOG.md)** — the complete three-day engineering log: the debugging journey, the wrong theories and what disproved them, the root cause, and ten process lessons. We kept the failures in on purpose.
- **[`TeamCode/ROOKIE_GUIDE.md`](TeamCode/ROOKIE_GUIDE.md)** — how new members use this codebase (also shared with our organisation's junior teams).
- **Git history** — dated, incremental evidence of the development process.

## The team

Team 37014 is the advanced team in a five-team organisation (junior,
girls, eastern-division, and advanced programs) that shares knowledge,
tooling, and this documentation approach. Our software was written
solo by one year-8 student over a holiday weekend, then hardened
through team testing sessions.

---

*Contact: via our GitHub organisation, 37014FTC.*
