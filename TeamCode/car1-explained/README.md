# Car 1 code — overview and evaluation

This folder explains, line by line, the Car 1 robot code that arrived in `send-kingsley-20261112.zip` and now lives, **unmodified**, in:

```
TeamCode/
├── car1.xml                                  ← robot configuration (explained below)
├── README.md                                 ← the README that came in the zip (unmodified)
├── src/main/java/org/firstinspires/ftc/teamcode/
│   ├── Car1TeleOp.java        (563 lines)    ← match TeleOp
│   ├── Car1HardwareTest.java  (192 lines)    ← one-device-at-a-time tester + servo tuner
│   ├── Car1PPTest.java        (292 lines)    ← Pinpoint odometry self-test
│   ├── Car1AutoV2Base.java    (244 lines)    ← shared auto engine (not on Driver Station)
│   ├── Car1AutoShoot.java     ( 26 lines)    ← main auto: shoot + leave + park
│   ├── Car1AutoPark.java      ( 19 lines)    ← backup auto: leave + park
│   └── ServoStore.java        ( 56 lines)    ← servo values file on the Hub
└── car1-explained/                           ← you are here
```

One walkthrough per file: [Car1TeleOp](Car1TeleOp.md) · [Car1PPTest](Car1PPTest.md) · [Car1AutoV2Base](Car1AutoV2Base.md) · [Car1HardwareTest](Car1HardwareTest.md) · [Car1AutoShoot](Car1AutoShoot.md) · [Car1AutoPark](Car1AutoPark.md) · [ServoStore](ServoStore.md)

---

## How the pieces fit together

```
                    /sdcard/FIRST/car1_servo.txt  (on the Control Hub)
                              ▲ writes                │ reads at start
              Car1HardwareTest ┘          Car1TeleOp ◄─┴─► Car1AutoV2Base
              (tune open/closed,              ▲                  ▲
               save to file)                  │                  │ extends
                                        Car1AutoShoot    Car1AutoPark
                                        (shoot+park)     (park only)

     Pinpoint odometry "pp" ── used by ── TeleOp (display), PP Test (diagnosis), Auto base (driving)
     Same drive-motor mix (drive(y,x,r)) and glitch filter (readPose()) appear in TeleOp, PPTest, AutoV2Base
```

- **ServoStore** is the memory: Hardware Test discovers servo numbers → saves them on the Hub → TeleOp and the autos load them at start, falling back to constants if nothing is saved.
- **Car1AutoV2Base** is a template: it runs the whole auto (init menu → shoot → waypoints → log) and asks each subclass just three questions: `path()`, `shoots()`, `title()`.
- **Car1PPTest** is the evidence source: the "X and Y pods both REVERSED" setting and the bad-read glitch filter used everywhere were established with it on 11-12.
- All five op modes agree on: motor direction constants, the mecanum wheel mix, Pinpoint setup, try/catch device lookup ("not found" never a crash), and Hub-file logging.

## What "line by line" means here

Each walkthrough file quotes the source in order, chunk by chunk, and explains **every line** — including imports, comments, and closing braces — with FTC jargon (OpMode, telemetry, RunMode, PIDF, edge detection, bulk caching…) defined the first time it appears. The code itself was not touched; these notes live only in this folder.

---

## car1.xml — line by line

The robot **configuration file**: a description of what is plugged into which Hub port. The Driver Station loads it (Configure Robot → activate `car1`), and the `name=` attributes are exactly the strings the code passes to `hardwareMap.get(...)`.

```xml
<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
```
- **Line 1** — Standard XML header: version, character encoding, no external definitions needed.

```xml
<Robot type="FirstInspires-FTC">
```
- **Line 2** — Root element: one FTC robot configuration.

```xml
    <LynxUsbDevice name="Control Hub Portal" serialNumber="(embedded)" parentModuleAddress="173">
```
- **Line 3** — The USB "portal" through which the phone/Driver Station talks to the REV hubs. On a Control Hub this portal is built in — hence `serialNumber="(embedded)"` — and its own address is 173.

```xml
        <LynxModule name="Expansion Hub 2" port="2">
            <goBILDA5202SeriesMotor name="flywheelNectar" port="0" />
            <goBILDA5202SeriesMotor name="flywheelPollen" port="1" />
            <goBILDA5202SeriesMotor name="intake" port="2" />
        </LynxModule>
```
- **Line 4** — The second hub, daisy-chained on port 2 of the chain. Its three motor ports:
- **Line 5** — Port 0: the NECTAR (big ball) flywheel, named `flywheelNectar`. The element type `goBILDA5202SeriesMotor` tells the SDK the motor model so it can apply the right default PIDF and encoder assumptions.
- **Line 6** — Port 1: the POLLEN (small ball) flywheel.
- **Line 7** — Port 2: the intake roller.
- **Line 8** — End of the Expansion Hub's device list.

```xml
        <LynxModule name="Control Hub" port="173">
            <goBILDA5202SeriesMotor name="lb" port="0" />
            <goBILDA5202SeriesMotor name="lf" port="1" />
            <goBILDA5202SeriesMotor name="rf" port="2" />
            <goBILDA5202SeriesMotor name="rb" port="3" />
            <Servo name="sorterGateL" port="0" />
            <Servo name="sorterGateR" port="1" />
            <Servo name="sorterEndL" port="2" />
            <Servo name="sorterEndR" port="3" />
            <ControlHubImuBHI260AP name="imu" port="0" bus="0" />
            <goBILDAPinpoint name="pp" port="0" bus="0" />
        </LynxModule>
```
- **Line 9** — The main Control Hub itself, at address 173.
- **Lines 10–13** — The four drive motors: `lb` (left back) on 0, `lf` (left front) on 1, `rf` on 2, `rb` on 3 — note the physical port order is b/f/f/b, but only the *names* matter to the code.
- **Lines 14–17** — Four servos: left/right sorter gates (the ball feeds) and left/right sorter ends (future end exits — not fitted on Car 1 yet).
- **Line 18** — The Control Hub's **built-in** IMU (the BHI260AP chip), named `imu`, on internal I2C bus 0.
- **Line 19** — The goBILDA **Pinpoint** odometry computer, named `pp`, on the **same I2C bus 0** — two devices can share one bus because each has a unique address; this is normal for Pinpoint + built-in IMU.
- **Line 20** — End of the Control Hub's list.

```xml
    </LynxUsbDevice>
</Robot>
```
- **Lines 21–22** — Close the portal and the robot config.

**Why names are the whole contract:** the README of the zip states the rule — if a cable is in the wrong *port* but the config name is right, the program still works; if a device is missing entirely, the code shows "not found" instead of crashing. That is possible only because every `hardwareMap.get(...)` in all five programs is wrapped in try/catch.

---

## Overall evaluation

### What this codebase does well

1. **Robustness as a philosophy.** Every hardware lookup can fail safely; every file operation is try-with-resources and never crashes the robot; a missing Pinpoint stops the auto *before* it moves. For a competition robot, "never crash, always report" is the right instinct.
2. **Honest documentation.** Constants carry dates and provenance ("11-11 tested on robot: intake was backwards", "not tested", "estimated from the field drawing"). You can tell tested facts from guesses by reading the comments — rare and valuable.
3. **Real iterative engineering.** The v3 changes (gate wait fallback after the ±3 % starvation at 5500 rpm, the removed spit mode, the always-3800/5500 decision) are the code's own history of problems found on the robot and fixed. The glitch filter exists because a logged incident was investigated.
4. **Correct timing discipline.** Bulk caching for loop speed; capped `dt` for the ramp; auto budgets (29 s total, 6 s/leg, 20 s shooting, 3 s spin-up, 1.5 s gaps); one-shot rumble flags.
5. **Good structure for a small team.** The autos are pure data over one shared engine; ServoStore decouples tuning from code; Hardware Test + PP Test are purpose-built tools that the match code depends on, not toys.

### Watch-outs before competition

1. **The two autos have never driven on a robot.** Road: small-space (Y) run → empty-field full run → check the log's "reached / NOT reached" lines → then trust them. Waypoints are drawing estimates (tile = 609.6 mm).
2. **Auto doesn't ramp the flywheel** (TeleOp ramps ≥1 s to protect printed couplers; `shoot()` commands full speed instantly). If the couplers are fragile, that asymmetry matters.
3. **Dead code kept on purpose**: `matchSpeed` targets, `loadFly()`/`saveFly()`, `UNJAM_TAP_S` — harmless, but know they do nothing.
4. **Hub orientation constants are guesses** → field-centric heading may be rotated; fix by measuring or `USE_PP_HEADING = true`.
5. **Duplication risk**: the drive mix, Pinpoint setup, and glitch filter are copy-pasted across TeleOp / PPTest / AutoV2Base. They agree today; any future edit must be made in three places (four counting offsets) or they'll drift.
6. **Pinpoint offsets (15, 55) are from the old chassis** — only distorts position while turning, but re-measuring is cheap insurance for the autos.
7. **Untested numbers to tune on the robot**: `GATE_PULSE_MS = 350` (one ball per pulse?), `FEED_MS = 250`, and the shot-detector's 3 s give-up window.

### Nothing was changed

Per instructions, every file was copied byte-for-byte; these documents and this folder are the only additions.
