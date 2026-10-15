# Team 100 Glossary

Team 100 vocabulary, for humans and for AI assistants. Where a word means
something different here than it does in generic FRC, that is called out.

If you are an AI assistant, read this file at the start of a session, before
you answer anything, so you are not guessing at or looking up words the student
uses in passing. `AGENTS.md` is the policy; this is the vocabulary. When a
student uses one of these terms wrongly, correct it the way `AGENTS.md` says to
correct anything else: have them check it, and do not let it stand.

## This repository

- **all26** — the monorepo holding *all* Team 100 code for the 2026 season.
  One repo so library code is easy to share. `all25` and `all24` are the
  archived previous years. The `2027` branch builds alongside `main`.
- **comp** — the RoboRIO competition code for this year's robot.
- **lib** — evergreen library code, reused every year. Its own Gradle project.
  Year-specific code does not belong here; it goes in `comp` or a study.
- **studies** — small independent projects, one folder each, each its own
  Gradle project. This is where most students work.
- **a study** — a project to investigate one specific thing, separate from the
  competition code. Created with the WPILib "New Project" wizard
  (Template > Java > Command Robot, Desktop Support checked) inside `studies/`,
  then given a `.code-workspace` that includes both the study folder and
  `../../lib`. See `lib/doc/getting_started/STUDY.md`.
- **console** — the button board: hardware and code.
- **raspberry_pi** — coprocessor tasks that do not run on the RoboRIO: vision,
  logging, GTSAM.

## Team 100 concepts (these differ from generic FRC usage)

- **Mechanism** — a Team 100 wrapper around a *motor*, an *encoder*, and
  *gears*. Its job is to encapsulate the gear ratio: the phrase "gear ratio"
  should not appear anywhere outside a mechanism.
- **Servo** — a Team 100 servo is a mechanism plus closed-loop control, for a
  single degree of freedom. Give it a goal and it goes there. It is **not** a
  hobby RC servo. For multiple coordinated axes, write a custom `Subsystem` and
  use the `Mechanism` layer inside it, not `Servo`.
- **Identity** — an enum of RoboRIO serial numbers. Factories switch on it so
  one codebase can drive several different robots. If code behaves differently
  on two robots, this is usually why.
- **Experiment** — a named switch that changes behavior at runtime, each with
  its own dashboard widget. Some are temporary, some are permanent: turning off
  `HeedVision` makes the robot ignore the cameras.
- **coherence** — Team 100's discrete clock and observation cache, so that
  everything sampled within one time step represents the same instant. It
  exists because the WPILib clock is continuous and Java timing jitter
  otherwise leaks into anything that differentiates or steps a profile.
- **framework** — near-copies of WPILib classes with better logging.
  `TimedRobot100` is `TimedRobot` that also logs loop overruns.
- **the "100" suffix** — how we name a class whose name is already taken by
  WPILib. WPILib has `MecanumDrive`, so ours is `MecanumDrive100`.
- **logging** — loggers are passed in through constructors, so the
  NetworkTables tree mirrors the construction graph.
- **reference / setpoint** — a *reference generator* takes a profile or a
  trajectory and produces the "current" and "next" setpoints, time-aligned
  through `coherence`.
- **profile vs. trajectory** — a **profile** is constrained motion in 1d or
  2d-with-heading; cheap, and the default for most motion. A **trajectory** is
  a spline path with a precomputed timing schedule, for curving around known
  obstacles. Reach for a profile unless the path needs the curve.
- **localization** — full-field pose from camera sightings of AprilTags plus
  odometry and gyro. **targeting** turns camera observations into
  field-relative targets.
- **hid** — human interface devices: wrappers for controllers and the console.
- **indicator** — ways to get the drive team's attention (lights, sounds).
- **Friction**, **CurrentLimit**, **PIDConstants** — the motor configuration
  classes in `lib.config` that show up as constructor arguments on a motor.
  `PIDConstants` uses SI units. These are the arguments students most often
  copy without understanding; see rule 6.

## Package name suffixes

Package and class names encode the shape of the mechanism. Reading them saves
a lot of searching:

- **P** = a prismatic joint (slides), **R** = a revolute joint (rotates), in
  order along the kinematic chain. So `pr` is prismatic-then-revolute, `rrr` is
  a three-jointed arm, `pprrr` is two slides then three rotations.
- **r1**, **rn** — one, or n, revolute degrees of freedom.
- **se2** — a planar pose: x, y, and heading. This is the SE(2) Lie group, and
  the reason `Pose2d` composition is not just adding three numbers.
- **se3**, **six_dof** — full 3d pose.
- **so2**, **so3** — rotation only, in 2d and 3d.

## Conventions students are expected to follow

From `lib/doc/getting_started/Conventions.md`:

- Locals are `camelCase`; members are `m_` prefixed; constants are
  `UPPER_SNAKE`; packages lower case, classes title case.
- `final` liberally, minimum visibility, favor immutability.
- Small classes, short methods; names describe meaning, and if the meaning
  changes the name changes.
- Four spaces, never tabs, use the auto-formatter.
- Comments say *why*, not *what*; Javadoc on the public surface.

## VS Code, which is where they work

- **the command palette** — `Ctrl+Shift+P`. Where the WPILib commands live:
  "Build Robot Code", "Deploy Robot Code", "Simulate Robot Code".
- **the WPILib extension / the W icon** — the top-right button that opens the
  same command list.
- **the Explorer, Search, Source Control, Testing panels** — the icons down the
  left bar: files, `Ctrl+Shift+F` search, git, and the test runner (beaker).
- **the Terminal/Output panel** — the bottom pane where build output appears.
  Reading it is a skill; driving it is not required.
- **a workspace (`.code-workspace`)** — a VS Code file listing more than one
  folder. Every study has one, so that the study and `lib` are both open.
- **Gradle** — the build tool VS Code runs underneath. Each of `comp`, `lib`,
  `raspberry_pi`, and every study is its own Gradle project, which is why you
  have to pick one when you build.

## Hardware and tools they will name

- **RoboRIO** — the robot's main controller; runs `comp` or a study.
- **Kraken X60 / TalonFX / Phoenix 6** — CTRE motors and their vendor library.
  **NEO / SparkMax / REV** — the other vendor's.
- **CAN ID** — the address of a device on the CAN bus; duplicated IDs are a
  classic cause of "the motor does nothing."
- **LaserCAN** — time-of-flight distance sensor. **Redux** — the gyro.
- **AprilTag** — the fiducial markers on the field used for localization.
- **Driver Station / FMS** — the laptop app that enables the robot, and the
  field system that controls it at a competition.
- **teleop / auton** — the driver-controlled and autonomous match periods.
  `teleopPeriodic` runs about every 20 ms during teleop.
- **NetworkTables** — the robot's key/value bus to the dashboard.
  **AdvantageScope** and **Glass** are what they look at it with.
- **GTSAM** — the factor-graph library used for localization on the Pi.
