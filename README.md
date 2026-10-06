# 🤖 37014 FTC — Team Code

This is our robot's brain. Everything the robot does lives here: the
driver controls, the autonomous missions, and the settings that make it
all work.

**New here? Read this page top to bottom. That's the whole briefing.**

---

## Robot words, in plain English

| Word | What it actually means |
|---|---|
| **OpMode** | One program for the robot. You pick it from a list on the Driver Station. |
| **TeleOp** | A program where a human drives with gamepads. |
| **Autonomous** | A program where the robot drives itself. |
| **Deploy** | Copying our code from the laptop onto the robot. Happens every time we change code. |
| **Driver Station (DS)** | The phone/app with the big INIT and START buttons. |
| **Control Hub** | The big brick on the robot that runs everything. |
| **Pinpoint** | The little board that always knows where the robot is (odometry). |
| **INIT → START** | INIT = get ready (always press it first!). START = go. |
| **STOP** | The panic button. Also ends seizures. |

---

## The programs (what to pick on the Driver Station)

### 🎮 Driving (TeleOp section)

| Name | What it does | Who it's for |
|---|---|---|
| `37014_ArcadeTeleop` | Left stick moves ANY direction, right stick turns | Main drivers |
| `37014_FieldCentric` | Same but controls follow the field, not the robot | Main drivers |
| `37014_FieldMacro` | Field-centric **+ press X to auto-drive to a spot** | Advanced |
| `37014_TankTeleop` | One stick per side, like tank tracks | Old school |
| `37014_ArcadeRookie` / `37014_FieldRookie` | Same as the big ones, but speed-limited | **You, probably** 😄 |

**Every teleop:** hold **A** for slow mode. Trust us, use it indoors.

### 🤖 Self-driving (Autonomous section)

| Name | What it does |
|---|---|
| `37014_DiagonalNW` | Goes 1 ft diagonally northwest while rotating 90° — all in one motion |
| `37014_DiagonalNW_x2` | Does that twice, zigzag style |
| `37014_ABC` | Goes A → B (diagonal + rotate) → C |

### 🔧 Tests (ask a year 8 before running these)

| Name | What it does |
|---|---|
| `37014_MotorTest` | Spins one wheel at a time (X/A/Y/B) — **wheels off the floor!** |
| `37014_OdometryRuler` | Push the robot by hand, it shows how far you went |
| `37014_FrameProbe` / `37014_OdometryCheck` | Robot self-checks for diagnosing problems |

---

## Rules of the road

1. **First run of anything new = wheels off the floor.** Prop the robot on a block. Non-negotiable.
2. **Never edit the numbers in `TeamCode/.../pedro/Constants.java` without a year 8.** That file is the robot's tuning — wrong numbers have caused *thousands* of seizures (see PROJECT_LOG.md, "The Hall of Seizures").
3. **The robot once tried to enter an elevator by itself.** Watch it when it's self-driving. Finger near STOP. Always.
4. If a program crashes, the Driver Station shows a red message — screenshot it and tell a year 8.

---

## Getting the code (for your own laptop)

1. Ask a year 8 to add your GitHub account to the team repo (Settings → Collaborators).
2. Click the green **Code** button → **Open with GitHub Desktop** (or copy the link).
3. Open the folder in **Android Studio** (free). First sync needs internet — let it finish, it takes ages. That's normal.

To put code on the robot: USB cable from laptop to Control Hub → press the green **Run ▶** button in Android Studio. That's a "deploy."

---

## Where the deep stuff lives

- **`TeamCode/PROJECT_LOG.md`** — the full story of how this robot went from
  "how does it move" to what it is now, including every mistake. Genuinely
  worth reading, even (especially) the failure parts.
- **`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`** — all our
  programs. Names start with `37014_`.
- **`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`**
  — the robot's tuning (see rule 2).

## Who to ask

The year 8s — one of them is the software captain who built this repo. No
question is too basic — nobody on this team knew what an OpMode was a
week ago either.

---

*Last updated: Oct 2026. If this page is wrong, fixing it is your first
open-source contribution. 🎉*
