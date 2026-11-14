# Car1AutoPark.java — line by line

**File:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Car1AutoPark.java` (19 lines)
**What it is:** the backup autonomous. It scores only the "leave the start + park in the loading zone" points — no shooting. It's deliberately tiny because *all* the real logic lives in its parent class `Car1AutoV2Base`; this file just supplies the three things that make it different from the other auto.

---

## Line 1 — package

```java
package org.firstinspires.ftc.teamcode;
```
- **Line 1** — The folder address of this class (same package as every other team file, so they can see each other without imports).

## Lines 3 — import

```java
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
```
- **Line 3** — Brings in the `@Autonomous` annotation (see line 12). Op modes are in a different package than team code, hence the import.

## Lines 5–11 — the header comment

```java
/**
 * Backup auto: only LEAVE + PARK (8 points). Works for red and blue (field is point-symmetric about its center).
 * Start: back of robot against the wall at your own driver area, front of robot facing into the field, parked next to
 * the LOADING ZONE (the LOADING ZONE is on the robot's left; the robot must not touch the LOADING ZONE tape).
 *   1 forward 200 away from the wall (LEAVE)
 *   2 550 to the robot's left, so part of the robot is in the LOADING ZONE (PARK)
 */
```
- **Lines 5–11** — Documents the whole strategy: where to place the robot before INIT, and the two moves. "Point-symmetric" means: rotate the whole field 180° and red becomes blue — so the *same* moves work for both alliances when they're described relative to the robot. The `1` / `2` numbers are the waypoints, defined on line 15.

## Line 12 — the @Autonomous annotation

```java
@Autonomous(name = "Car1 Auto Park Only", group = "Car1", preselectTeleOp = "Car1 TeleOp")
```
- **Line 12** — An *annotation* is a label attached to the class. The FTC system reads this one to register the program:
  - `name = "Car1 Auto Park Only"` — the title shown on the Driver Station's op mode list.
  - `group = "Car1"` — the folder/section it is sorted into on the Driver Station.
  - `preselectTeleOp = "Car1 TeleOp"` — after this auto finishes (or is stopped), the Driver Station automatically switches its selection to *Car1 TeleOp*, so the drivers can't forget to pick TeleOp after the autonomous period. The name here must exactly match the TeleOp's registered name.

## Line 13 — the class declaration

```java
public class Car1AutoPark extends Car1AutoV2Base {
```
- **Line 13** — `extends Car1AutoV2Base` = inheritance: this class is `Car1AutoV2Base` plus the three overrides below. All the driving, odometry, init-menu and logging machinery is inherited, not copied. (This pattern is called a *template method*: the base class runs the show and asks "give me your path / do you shoot / what's your title?")

## Lines 14–16 — override #1: the path

```java
    @Override protected double[][] path() {
        return new double[][]{{200, 0, 0}, {200, 550, 0}};
    }
```
- **Line 14** — `@Override` says "I'm replacing a method my parent class declared abstract". `protected` = visible to this class and its family. `double[][]` = a 2-D array: an array of waypoints, each waypoint an array of numbers.
- **Line 15** — The two waypoints, in **millimetres and degrees**, in the Pinpoint's coordinate frame where the start pose is the origin, +x = forward from where the robot starts, +y = to the robot's left, angle = counter-clockwise:
  - `{200, 0, 0}` — drive **200 mm straight forward** (away from the wall). LEAVE done.
  - `{200, 550, 0}` — keep the same x, move **550 mm to the left**, into the LOADING ZONE. PARK done.
  - The final `0` in each group is the target heading in degrees (0 = don't turn).
  - Why forward is +x for a robot whose back is against the wall: the robot was *placed* facing into the field, so "robot forward" is "into the field" — the same for red and blue.

## Line 17 — override #2: does it shoot?

```java
    @Override protected boolean shoots() { return false; }
```
- **Line 17** — `false` = this auto never spins the flywheel. In the base class this is what decides whether to grab the flywheel motor + right gate servo, whether the "how many balls" menu option appears, and whether the shooting phase runs.

## Line 18 — override #3: the title

```java
    @Override protected String title() { return "Car1 Auto Park Only (back of robot against your driver-area wall, LOADING ZONE on the robot's left)"; }
```
- **Line 18** — The text the base class prints at the top of the Driver Station during INIT — a built-in reminder of how to place the robot, so the person setting up doesn't need the README.

## Line 19 — end

```java
}
```
- **Line 19** — Closes the class.

---

## Evaluation

**Strengths**
- Model example of inheritance done right: 3 one-line decisions fully describe a distinct autonomous; zero duplicated logic.
- The placement instructions are baked into the code (`title()`), which is where they're actually needed at the field.
- `preselectTeleOp` removes a classic rookie match-day mistake.

**Watch-outs**
- Waypoints are the estimated values from the field drawing (per the zip's README) — same as the main auto, they must be verified on a real field before a match.
- The whole auto trusts the Pinpoint's start pose — moving the robot between INIT and PLAY silently ruins it (the base class warns about this on screen).
