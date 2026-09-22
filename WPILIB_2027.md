# BLine-Lib 2027 beta

This development candidate targets **WPILib 2027.0.0-alpha-7 and Java 25**.
It is pending manual review and distribution. The stable WPILib 2026 release
remains on [`main`](https://github.com/edanliahovetsky/BLine-Lib).
Later WPILib alphas require validation before being called supported.

## Robot setup

The year-specific `BLine-Lib-2027.json` describes one Java artifact containing
both adapters. Your robot project supplies its chosen command framework;
BLine's published dependencies do not install either framework for you.

| Robot project | Builder | Drive requirement | Scheduler |
| --- | --- | --- | --- |
| Commands v3 | `FollowPath.Builder` | `Mechanism` | `Scheduler.getDefault()` |
| Commands v2 | `FollowPathV2.Builder` | `Subsystem` | `CommandScheduler.getInstance()` |

Both constructors require, in order:

```java
import frc.robot.lib.BLine.commands.FollowPath;
import frc.robot.lib.BLine.following.DriveType;
import frc.robot.lib.BLine.path.Path;

var paths = new FollowPath.Builder(
    DriveType.TANK, drive,
    drive::getPose, drive::resetPose,
    drive::getMeasuredRobotRelativeVelocity, drive::driveRobotRelative,
    translationPID, rotationPID, crossTrackPID
).withDefaultShouldFlip();

var auto = paths.build(new Path("score")).withPoseReset();
```

Use `DriveType.SWERVE`, `MECANUM`, or `TANK` for the actual robot. The path
file does not select the robot's drivetrain. The callbacks use WPILib's
`Pose2d` and `ChassisVelocities`, with **robot-relative m/s and rad/s** for
velocity feedback and output. Your drivetrain still applies its kinematics,
wheel/module control and wheel-speed saturation.

Translation and cross-track PIDs take metres and produce m/s; the heading PID
takes radians and produces rad/s. BLine resets all three controllers at every
execution and applies the path's endpoint tolerances. Controller gains,
integral settings and period remain under team control. Configure continuous
heading behavior through the follower; it enables the controller's wrapped
input. Use distinct controllers for drivetrains that can run concurrently.
No track width is required by BLine's chassis limiter. Tank kinematics still
need track width in your robot code.

For a TimedRobot v3 project, call `Scheduler.getDefault().run()` from
`robotPeriodic()`, schedule `auto` in `autonomousInit()`, and cancel the
appropriate commands when autonomous ends. V2 uses its `CommandScheduler`
equivalents. OpModeRobot projects use WPILib's op-mode lifecycle and scheduler.
Use the same scheduler consistently for followers and their registered events.

A v3 sequence can await each path without retaining its drive requirement:

```java
var auto = Command.noRequirements(co -> {
    co.await(paths.build(pathA).withPoseReset());
    co.await(paths.build(pathB));
}).named("Collect and score");
Scheduler.getDefault().schedule(auto);
```

V2 retains `BLineCommandsV2.sequence(...)` and its other proxy composition
helpers. Use `org.wpilib.command3` or `org.wpilib.command2` imports for the
selected framework, and `org.wpilib` imports for WPILib 2027 types.

## Execution options and mutable paths

A builder holds robot wiring and an optional shared flip policy. `build(path)`
returns a new command. `withPoseReset()`, `withShouldFlip(BooleanSupplier)`,
`withShouldMirror(BooleanSupplier)` and `withTankDriveDirection(...)` modify
only that command; they do not configure the builder or subsequent commands.
Capture their fluent return values, as required by WPILib's command checker.

Every execution validates and copies the current source `Path`, samples its
transform policies once, resets pose when requested and eligible, then captures
the measured starting pose. Editing the source affects the next run, never an
active run. Files are not reread automatically.

`path.setFlipped(boolean)` and `path.setMirrored(boolean)` express desired
states and are idempotent. `isFlipped()` and `isMirrored()` report those states.
Use these instead of repeated geometric toggles.

A path may contain one translation-bearing destination or begin with a rotation
or event. BLine supplies the measured starting pose for that execution. With no
authored start, `withPoseReset()` warns once and skips reset. The editor's grey
preview-start robot supplies an equivalent movable start for its ideal preview;
it is private editor metadata and is not an authored robot-path element.

For a manual reset, use `path.getAuthoredStartPose().ifPresent(drive::resetPose)`.
It returns empty for a current-pose start; `getStartPose()` throws in that case.
The overload taking a `Rotation2d` supplies the heading for a translation-only
authored start. A starting waypoint supplies its own heading. For module
preorientation without an authored start, pass `drive::getPose` to
`getInitialModuleDirection(...)`.

Load JSON with `new Path("score")`, `new Path(projectDirectory, "score")`, or
`Path.fromJson(json, defaults)`. Structural file errors name the file and
relevant element/field. Executable validity is checked again when following,
so programmatic edits are included. The former `JsonUtils` and resolved constraint records
are internal; use `Path` APIs for loading/editing. Missing optional constraints retain
their defaults; present malformed values report the constraint and field. Rejected
loads leave shared project defaults unchanged.

`path.isValid()` checks the current elements and resolved constraints using the
execution validation, without logging or changing the path. Construction and
editing remain permissive. `reorderElements(...)` requires each existing index
exactly once and rejects invalid orders without changing the path.

## Handoffs and endpoints

`HandoffMode.RADIUS` uses distance to the target. `HandoffMode.PROGRESS` uses
projection along the incoming segment; crossing its threshold does not require
staying within the displayed gate's lateral endpoints. In either mode the
handoff distance is measured in metres.

Resolution is **element → path → project default → Radius**:

```java
var target = new Path.TranslationTarget(3, 2)
    .withHandoffMode(HandoffMode.PROGRESS)
    .withHandoffDistanceMeters(.3);
path.setHandoffMode(HandoffMode.PROGRESS); // Optional path default
path.clearHandoffMode();                 // Inherit project default
Path.setDefaultHandoffMode(HandoffMode.RADIUS);
```

JSON stores `handoff_mode` on the path or translation target and
`default_handoff_mode` in project kinematic constraints. Values are `radius`
and `progress`. Existing `intermediate_handoff_radius_meters` distance keys
remain supported in both modes. Shared project defaults remain shared; each
execution resolves them once. Loading another project configuration changes
subsequent runs' project defaults.

Rotation interpolation uses ordered geometric progress independently of early
translation handoffs. A join continues from the previously requested heading;
PID and slew limits still control actual motion. There is no trajectory or
trapezoidal motion-profile controller.

With zero final minimum translation velocity, translation is commanded to zero
at position tolerance, and the path completes when the applicable heading tolerance
is also met. Completion does not wait for measured or commanded velocity to settle. With a positive final minimum, the path completes at
the position tolerance with best-effort rotation and retains its achieved
velocity output for the next command. It does not jump instantly to the
minimum. Cancellation and faults stop output. Teams must deliberately handle
what runs after a rolling exit.

Tank ignores intermediate authored rotations. At position tolerance it switches
directly to the last waypoint's body heading, with zero forward speed. A rolling tank exit skips that
stationary alignment. Swerve/mecanum can rotate while translating.

Tank limiting selects forward speed and turn rate jointly by the approximately closest next
velocity vector, subject to angular slew and the combined forward/cornering
acceleration limit. It does not prioritize turn rate or automatically lower the
speed request for tighter turns. The editor uses the same limiter with ideal,
PID-free guidance. This one-step choice can sustain a saturated turn instead of
temporarily turning less to make room for braking; test your paths and controller
tuning, especially high-speed bends. It is not a guarantee of path convergence.

## Tank driving direction

The editor's **Drive forward** / **Drive backward** buttons save exported robot
behavior. Preview and generation use that same direction:

```json
"tank_drive_direction": "backward"
```

Resolution is **explicit command override → saved path direction → Forward**.
Older files without the property remain Forward. The editor migrates an older
`preview.tank_direction` only when the new property is absent, then removes the
duplicate metadata value. Invalid JSON direction values produce a loading error.

```java
var saved = paths.build(path);
var forward = paths.build(path).withTankDriveDirection(DriveDirection.FORWARD);
var backward = paths.build(path).withTankDriveDirection(DriveDirection.BACKWARD);
path.setTankDriveDirection(DriveDirection.BACKWARD);
DriveDirection direction = path.getTankDriveDirection();
```

Backward means rear-first travel through the same element order. It changes
neither authored headings nor transforms nor reset behavior. Swerve/mecanum
ignore the saved property; explicitly requesting Backward on a holonomic
command is an invalid configuration.

## Event commands

Register keys during robot setup, using the selected framework adapter:

```java
FollowPath.registerEventTrigger("intake", intakeCommand);
FollowPath.registerEventTrigger("setGoal", () -> intake.setGoal(goal));
```

Reached events are queued in path order and dispatched by the default scheduler
outside the follower, normally one cycle later. Final-segment events are retained
when a path finishes inside its endpoint tolerance. An event command can outlive
the follower and its surrounding path sequence. WPILib still enforces ordinary
requirements and scope rules; scheduling an already-running command does not
necessarily restart it. Runnable callbacks must not block the robot loop.

Canceling a follower removes only that execution's pending events. Already
scheduled commands belong to the robot application's lifecycle. On a whole-auto
abort or mode transition, a team using global cancellation can do:

```java
Scheduler.getDefault().cancelAll();
FollowPath.clearPendingEventTriggers();
```

V2 dispatches independently of the active button loop. It uses its scheduler and
`FollowPathV2.clearPendingEventTriggers()`.
Teams with other commands to preserve should cancel only their owned commands.

## Existing telemetry and field display

`paths.withTelemetry(table)` sends existing `FollowPath/...` signals to an
application-owned WPILib `TelemetryTable` in subsequently built commands.
For example, a robot with an existing DataLog can create its table with
`new TelemetryTable(new DataLogTelemetryBackend(log, ""))`.
Logging callbacks remain supported. This does not add a second logger,
tuning interface or new library metrics.

`BLineField.drawPath(field, path)` and its named overload still display paths
on `Field2d`. In WPILib 2027, use `org.wpilib.smartdashboard.Field2d` and the
new telemetry API to publish the existing widget with `table.log("Field", field)`
in your periodic logging. The robot review examples demonstrate framework
setup and lifecycle wiring.

## Imports and internal structure

The beta groups public types by responsibility:

| Package under `frc.robot.lib.BLine` | Public application API |
| --- | --- |
| `path` | `Path` and its nested elements/constraints, `HandoffMode`, `DriveDirection` |
| `commands` | `FollowPath`, `FollowPathV2`, `BLineCommandsV2` |
| `following` | `DriveType`, `HolonomicRateLimiter` |
| `field` | `BLineField`, `FlippingUtil` |

Update older flat-package imports to these packages. Builder arguments and fluent
execution options retain the same signatures. Both command adapters still share
one control implementation; swerve and mecanum share holonomic control.

The former `ChassisRateLimiter` utility is now
`frc.robot.lib.BLine.following.HolonomicRateLimiter`. Its `limit(...)` arguments
and limiting behavior are unchanged.

`PreparedPath` and `FollowerSession` are internal bridges between packages, not
supported configuration APIs for robot code. Their Java-public visibility is
needed across package boundaries. The parser, controllers, traversal helpers,
and event queue remain package-private. The former public `JsonUtils` and raw
resolved-constraint inspection APIs have no supported direct replacements; use
the authored `Path` API for loading and editing.
