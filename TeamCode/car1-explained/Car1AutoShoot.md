# Car1AutoShoot.java — line by line

**File:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Car1AutoShoot.java` (26 lines)
**What it is:** the main autonomous: shoot the preloaded POLLEN balls, leave the start (LEAVE), and drive to the LOADING ZONE (PARK). Exactly like `Car1AutoPark` it is a tiny subclass of `Car1AutoV2Base` — only the path, the "yes we shoot" flag, and the title differ.

---

## Line 1 — package

```java
package org.firstinspires.ftc.teamcode;
```
- **Line 1** — Same package as all team code, so no import is needed to see `Car1AutoV2Base`.

## Line 3 — import

```java
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
```
- **Line 3** — Provides the `@Autonomous` annotation used on line 19.

## Lines 5–18 — the header comment

```java
/**
 * Main auto: start with the front of the robot against the wall of the CELL that faces up in your alliance color,
 * lined up with the CELL -> shoot the preloaded POLLEN from where it stands -> LEAVE (back away from the wall)
 * -> drive along the side to the LOADING ZONE and PARK.
 * Works for red and blue (field is point-symmetric about its center). Start: front of robot against the wall,
 * robot in your own half, not touching the garden or flowers, not inside the LOADING ZONE.
 *
 * Path (Pinpoint coordinates, start is the origin, x toward the front of the robot +, y to the robot's left +, mm;
 * estimated from the field drawing, one tile = 609.6):
 *   1 back up 300 away from the wall (LEAVE)
 *   2 back up diagonally to x -560, 790 to the robot's right (to the column next to the HIVE stand, avoiding the HIVE)
 *   3 keep backing up to x -2450 (along this column to the row with the LOADING ZONE)
 *   4 move to the robot's right again to y -1130, so part of the robot is in the LOADING ZONE (PARK)
 */
```
- **Lines 5–18** — The game plan in words:
  - **Lines 6–8** — the full sequence and the start placement (front against the CELL wall, lined up with your colored CELL).
  - **Lines 9–10** — why one program works for both alliances (180° point symmetry), plus the legality constraints (own half, not touching garden/flowers, not already in the LOADING ZONE).
  - **Lines 12–13** — the coordinate convention and the honest note that waypoints are **estimates**: one floor tile is 609.6 mm, and the numbers were scaled off the field drawing — not measured.
  - **Lines 14–17** — the four moves, which map one-to-one onto the four waypoints on line 22: back off the wall, back diagonally to a safe column (avoiding the HIVE), drive down that column, slide into the LOADING ZONE.

## Line 19 — the @Autonomous annotation

```java
@Autonomous(name = "Car1 Auto Shoot+Park", group = "Car1", preselectTeleOp = "Car1 TeleOp")
```
- **Line 19** — Registers the program on the Driver Station as **"Car1 Auto Shoot+Park"**, filed under group **Car1**, and auto-selects **Car1 TeleOp** when the auto ends so the team can't start driver control with the wrong program.

## Line 20 — the class declaration

```java
public class Car1AutoShoot extends Car1AutoV2Base {
```
- **Line 20** — Inherits everything from the shared auto base class; this file only supplies the differences.

## Lines 21–23 — override #1: the path

```java
    @Override protected double[][] path() {
        return new double[][]{{-300, 0, 0}, {-560, -790, 0}, {-2450, -790, 0}, {-2480, -1130, 0}};
    }
```
- **Line 21** — Replaces the parent's abstract "give me your waypoints" method.
- **Line 22** — Four waypoints, `{x, y, heading°}` in mm/degrees, start pose = origin, +x = the direction the robot's front pointed at the start (toward the wall!), +y = robot's left:
  - `{-300, 0, 0}` — 300 mm **backwards** (x negative = away from the front, i.e. off the wall). LEAVE.
  - `{-560, -790, 0}` — continue to a point 560 mm behind the start **and** 790 mm to the robot's **right** (y negative): backs out diagonally to the clear column beside the HIVE stand.
  - `{-2450, -790, 0}` — same distance right (−790), much further back (−2450): straight down that column toward the driver-area end of the field. Only x changes, so this leg is purely backwards.
  - `{-2480, -1130, 0}` — a final 340 mm further right (−1130 vs −790) into the LOADING ZONE. PARK.
  - All headings stay `0` — the robot never turns during this auto; it drives the whole path "backwards/sideways" like a crab, which is exactly what a mecanum drive is good at.
- **Line 23** — Closes the method.

## Line 24 — override #2: does it shoot?

```java
    @Override protected boolean shoots() { return true; }
```
- **Line 24** — `true` = the base class should grab the POLLEN flywheel and right gate servo, show the "how many balls" menu option, and run its shooting phase (spin up, pulse the gate open once per ball) **before** driving the path.

## Line 25 — override #3: the title

```java
    @Override protected String title() { return "Car1 Auto Shoot+Park (start: front of robot against the wall, lined up with the CELL)"; }
```
- **Line 25** — The reminder text shown during INIT on the Driver Station.

## Line 26 — end

```java
}
```
- **Line 26** — Closes the class.

---

## Evaluation

**Strengths**
- The no-turns path is a smart choice for a young team: turning is the least reliable motion for odometry; straight + strafe legs keep the Pinpoint's heading error from ever mattering.
- All tuning risk is honestly documented in the header (waypoints estimated from the drawing, tile = 609.6 mm).

**Watch-outs**
- Per the zip's README this auto **compiled but never ran on the robot** — first stop is a `Y` small-space test run (15 % path) in an open room.
- Leg 2's diagonal move (−560, −790) is the one most affected by strafe calibration error (mecanum strafe usually travels a bit short/long of commanded); the base class's per-segment 6 s timeout will catch a bad leg and continue.
- Shooting happens first, while parked against the wall — if the flywheel needs more than 3 s to reach 5500 rpm, `shoot()` fires anyway after the cap (SPINUP_MAX_S) and may miss.
