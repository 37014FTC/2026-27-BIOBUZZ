# PROJECT LOG — The Three-Day Robot Saga
---

## TL;DR

Robot went from "how do I make it move" to: tuned Pedro Pathing follower,
two autonomous missions, six teleop modes, and a field-absolute waypoint
macro system. Total time: ~3 days. Total seizures: roughly 5000 (see
[The Hall of Seizures](#the-hall-of-seizures)). Root cause of almost all
of them: **one backwards-counting odometry pod.**

---

## The Coordinate System (final verdict)

- **(0, 0) = center of the field**, units = **feet** (field is ±6 ft each way)
- +X = north (away from driver wall at heading 0), +Y = WEST (left)
- Heading 0 = north, CCW positive (Pinpoint convention)
- Frame verified empirically by `37014_FrameProbe`

---

## Day 1 — Making It Move & Measuring the World

### Programs built
- `37014_FBSTest` — timed forward/back/spin + strafe. The beginning of
  everything. Taught the mecanum wheel-sign cheat sheet:
  forward `++++`, spin `++--`, strafe-right `+--+` (per side FL/BL/FR/BR).
- `37014_OdometryRuler` — push robot A→B, read distance from the
  goBILDA Pinpoint. Originally read the pods as hub motors; **rewritten**
  when we learned the pods plug into the Pinpoint over I2C.
- `37014_Modes` — gamepad-selectable routines incl. slide-until-touch
  with odometry-guided return (first closed-loop code).
- `37014_MotorTest` — X/A/Y/B spins one motor each; the tool that
  settled every "which wheel is lying" argument.

### Calibration archaeology
1. Pod offsets measured: parallel 50mm, perpendicular 30mm (after a first
   measurement that disagreed wildly — re-measured properly).
2. Spin test: pushing robot made X read negative → `xPodDirection REVERSED`.
3. More spin test: drifting left+backward during rotation → both offsets
   needed NEGATIVE signs (X pod is RIGHT of center, Y pod is BEHIND).

### AutoTune adventures
- The tuner page "didn't work" → actually lives at
  `http://192.168.43.1:10158` (**the port is the whole trick**), and
  Windows silently hops off the hub's WiFi to any network with internet.
- Registered all tuners in `Tuning.java` via `@Tuner` methods (the file
  literally said "// Tuners go here" — now it does).
- **Built `ForesightSteps`**: the stock Foresight tuner runs 11 steps in
  one go and LOSES EVERYTHING if stopped early (braking returns an empty
  list → `.get(0)` crashes). Ours: one step per run, results persisted to
  `/sdcard/FIRST/pedro_foresight.properties`, sanity gates, smaller
  default distances, final config generator.
- Stock heading tuner produced **garbage** (−3090, +2852, "hundreds" —
  sign flipping between runs). Diagnosis: its log-fit explodes when
  chassis vibration makes omega bounce (`ln(K−vel)` ≈ `ln(0)`).
  **Built bounce-proof step 0**: moving-average smoothing + tau from
  time-to-63% + sanity range gate. Output: kP ≈ 10.9, repeatable.
- The **elevator incident**: forward-braking test sprinted the robot
  toward an open elevator. Driver reflexes 1, robot 0. (The step also
  revealed the stock return-leg distance is hardcoded 12" — input can't
  shrink it.)
- X pod froze mid-session (stuck reading 275.5 in). Power cycle revived
  it. It would do this again. It is still on parole.
- Final hard-floor tuning session: **17 of 17 values**, physics-consistent
  (strafe = 86% of forward speed — textbook mecanum ratio).

---

## Day 2 — The Seizure Wars & the Root Cause

### The gain ladder of despair (all on a lying localizer)
| Gain | Result |
|---|---|
| 0.5 (placeholder) | full seizure |
| 9.45 (measured) | seizure |
| 0.165 | no rotation at all (below static friction) |
| −9.45 | full rotation, then seizure |
| −1.0 | turns east correctly(!), then seizes |
| −0.5 | seizure |

Also: `holdPointTranslationalScaling` experiment **broke path following
the whole way** (those fields affect more than hold mode) → reverted.
And `follower.stop()` alone leaves the last wheel powers **latched** —
robot spun forever after the program ended → explicit
`drivetrain.drive(zero, true)` at every ending since.

### Wrong theories (killed by observation, mostly the captain's)
1. "Pedro's Y axis is right-positive" → robot sailed NORTHEAST for
   NORTHWEST. (The captain: "I wanted it to go northWEST, not northEAST" —
   the single most valuable sentence of the project.)
2. "Heading units are degrees not radians" → 0.165 gain.
3. "Pinpoint firmware doesn't report angular velocity" → hand-spin test
   showed ±0.2 at rest (normal noise), then 25–178°/s while rotating
   (alive). Theory dead.
4. "Motors are swapped left/right" → full Mecanum Tuner + MotorTest
   re-verification: all four correct.
5. "The stock odometry test page shows nothing" → it never emits
   `result()`; rebuilt it to print verdicts to the AutoTune page AND to
   logcat (tag `37014OC`, read from the laptop over the USB cable
   with adb — results literally pulled through the wire).

### The root cause
`37014_OdometryCheck` (standalone, crash-proof, adb-logged) measured:
+strafe command → robot slides LEFT (verified from behind the robot,
vantage matters!) → localizer reads **y = −23.4**. Left must be +y.
**The Y pod was direction-flipped in the Pedro config.** Fix:
`yPodDirection = REVERSED`.

Everything falls out of that one bit: mirrored diagonals, the sign-flipping
heading gains, why negative gains "worked" (double-flip compensation), why
Foresight's velocity fusion went insane. With honest odometry:
**heading gain +0.3 converged on the first try.** All negative-gain
conclusions were void.

### Victory
`37014_DiagonalNW` final form: NW diagonal, 90° clockwise **in flight**
(heading sweep front-loaded to 70% of the path — rotation must finish
before arrival per the win condition: no rotation at endpoints), clean
motor-off ending. **IT GOT THERE.** Minor cosmetic stutter remains
(measured translational gains are path-hot; scale down someday if needed).

### Add-ons
- `37014_DiagonalNW_x2` — the mission chained twice (zigzag). Worked;
  landed ~20° short per rep → diagnosed as ramp-tracking lag
  (∝ rampRate/gain) → gain 0.8. Untested at time of writing.
- `37014_ABC` — A→B (NW + in-flight 90°) → C (forward drive east to
  the point north of A). Written, never run.

---

## Day 3 — Teleop Day

| OpMode | Scheme | Cap |
|---|---|---|
| `37014_TankTeleop` | sticks = tracks, trigger strafe | 80% |
| `37014_JoystickTeleop` | L drive, R slide, bumpers turn | 80% |
| `37014_ArcadeTeleop` | L stick holonomic (+diagonals), R turn | 100% |
| `37014_ArcadeRookie` | same, rookie cap | 80% |
| `37014_FieldCentric` | field-centric via Pinpoint heading | 100% |
| `37014_FieldRookie` | same, rookie cap | 80% |
| `37014_FieldMacro` | field-centric + waypoint macros | 100% |

All: A-button slow mode (40%), brake stops, `scale()` =
**clamp-then-scale** so combined inputs (drive+turn = wheel power 2.0)
cap at MAX_POWER instead of clipping back to 1.0.

### `37014_FieldMacro` — the flagship
- Field-absolute coordinates: **center = (0,0), feet**, re-based at INIT
  via `setPosition` with the declared start pose.
- **X button** → autonomous P-seek to waypoint (hand-rolled, the pre-Pedro
  DiagonalNW machinery — chosen over the Follower so manual/auto share
  raw motors cleanly).
- Safety trinity: any stick touch instantly cancels AUTO; 4 s timeout;
  mode always on telemetry.
- **Left bumper = driver re-zero**: point nose away from you, tap, and
  stick-forward = "away from me." Born from the request to make controls
  track the driver station's *signal* — which is physically impossible
  (WiFi carries no bearing). The button is the honest 95% solution.
  (The real someday solution: AprilTag vision.)

---

## The Hall of Seizures

In rough order of appearance:

1. Placeholder heading gain 0.5 → first full thrash
2. Measured 9.45 → bigger thrash
3. −9.45 → full rotation THEN thrash
4. −1.0 → correct rotation, thrash on arrival
5. −0.5 → thrash with criss-cross contamination
6. FrameProbe phase 3 at strong gain → one full rotation + seizure
7. "Still stuttering the full way" (hold-scaling poisoned path following)
8. "Stutters then rotates indefinitely" (stop() latch bug + low battery)
9. Approximately 4,992 more minor stutters, assorted judders, and one
   near-elevator experience

**Moral:** every single one traced to feedback through bad data or bad
gains — never once to the motors, which were exonerated four separate
times. The wheels were always innocent. The messenger kept getting shot.

---

## Hardware notes & open items

- **X pod cable is on parole**: died twice day 1 (power-cycle revivals),
  intermittent since. Reseat/replace before trusting AUTO in a match.
- Pod offsets: X −50mm (right of center), Y −30mm (behind center),
  both pods `REVERSED`, 4-bar type, Pinpoint name `pp`.
- Motors: `lf` `rf` `lb` `rb`, left side `REVERSE` — verified twice.
- Battery sag contaminates tuning (gains measured at ~14V). Charge before
  final tuning; 12.45V resting = "charge soon" zone.
- Motor powers above ~0.15 overcome static friction; below that, nothing.

## Lessons that cost us time (worth re-reading)

1. **One variable per run.** Every real diagnosis came from single-bit
   experiments; every mess came from stacked changes.
2. **Watch the robot, not the theory.** "It went northeast" and "it slid
   left" killed more wrong theories than any code read.
3. **Vantage point matters** — "left" from in front of the robot is a lie.
4. **Empty config objects are landmines, not defaults** (the
   "config variable has not been set" exception).
5. **Anything a power cycle fixes, vibration un-fixes.**
6. **Motors never un-command themselves** — always zero them explicitly.
7. **Sanity-check every tuner output**: "does this number make physical
   sense?" caught kP = −3090 before it ever touched the robot.
8. **Persistent measurement files** beat re-running 11-step tuners.
9. When the robot misbehaves identically on every surface, it's not the
   surface.
10. The robot attempting to enter the elevator is a valid form of
    feedback. Just not the kind you wanted.

---

## Commit history

```
d0bddc4  driver programs + Pedro wiring            (day 1)
a65ae19  DiagonalNW mission + Y-pod root cause     (day 2)
c34ee98  teleop suite + waypoint macro system      (day 3)
```

Branch: `add-forward-back-spin` (not yet merged to master, no remote yet).
