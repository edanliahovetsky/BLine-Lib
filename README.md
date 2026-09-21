<h1 align="center">BLine-Lib</h1>

<p align="center">
  <a href="BLine-Lib-2027.json"><img src="https://img.shields.io/badge/version-2027.0.0--beta.1-2563eb" alt="Development candidate 2027.0.0-beta.1"></a>
  <a href="BLine-Lib-2027.json"><img src="https://img.shields.io/badge/WPILib-2027.0.0--alpha--7-c1121f" alt="WPILib 2027.0.0 Alpha 7"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-BSD--3--Clause-0f766e" alt="BSD 3-Clause License"></a>
</p>

**BLine** is a rapid point-to-point autonomous path planning and tracking
library for FIRST Robotics Competition. It is made by students for students and
built around practical tuning, quick iteration, and rapid empirical testing in
time-constrained build-season environments.

> [!IMPORTANT]
> This is the `wpilib-2027` development line for WPILib 2027.0.0-alpha-7,
> with Commands v3 and v2, swerve, tank and mecanum support. The beta is pending
> manual review and distribution. Read the [2027 guide](WPILIB_2027.md).
> The `main` branch remains the stable WPILib 2026 release line.

**Quick links**

🚀 **[Open the hosted editor](https://bline-web.pages.dev/)** — create, tune, preview, and export BLine paths in the browser.

🖥️ **[BLine Web](https://github.com/edanliahovetsky/BLine-Web)** — current web and desktop editor.

💬 **[Chief Delphi Thread](https://www.chiefdelphi.com/t/introducing-bline-a-new-rapid-polyline-autonomous-path-planning-suite/509778)** — discussion, feedback, and announcements.

📚 **[Documentation](https://bline-docs.pages.dev/)** — stable WPILib 2026 guides, tutorials, and reference.

<p align="center">
  <img src="docs/readme/bline-web-demo.gif" alt="BLine Web editor GUI demo" width="900">
  <br><br>
  <img src="docs/cone-demo.gif" alt="BLine robot cone demo" width="900">
</p>

## Installation

The 2027 candidate uses `BLine-Lib-2027.json`. It requires Java 25 and the
WPILib command framework selected by your robot project. This development
candidate is not yet published; do not replace a working 2026 installation
with an unpublished beta vendordep. Local review builds use the packaged
artifact supplied with the review projects.

## Quick Start

```java
import frc.robot.lib.BLine.*;
import org.wpilib.math.controller.PIDController;

// Commands v3: drive implements Mechanism. Use FollowPathV2.Builder and
// a Subsystem for Commands v2. Both have the same robot callbacks and options.
var paths = new FollowPath.Builder(
    DriveType.SWERVE,
    drive,
    drive::getPose,
    drive::resetPose,
    drive::getMeasuredRobotRelativeVelocity,
    drive::driveRobotRelative,
    new PIDController(5, 0, 0),  // distance: metres -> m/s
    new PIDController(3, 0, 0),  // heading: radians -> rad/s
    new PIDController(2, 0, 0)   // cross-track distance: metres -> m/s
).withDefaultShouldFlip();

var path = new Path("score"); // deploy/autos/paths/score.json
var auto = paths.build(path).withPoseReset();
```

The gains above are examples; tune them for your robot. The reset callback is
required, but reset is only enabled on commands with `withPoseReset()`.
Robot-relative velocity feedback must come from measured motion.
See the [2027 guide](WPILIB_2027.md) for scheduling, events, tank direction,
path defaults, endpoint behavior and telemetry.

### Rotation Override

For paths that need another system to own robot heading temporarily, such as
vision aiming while translating along a path, override the path follower's
rotational output:

```java
FollowPath.overrideRotation(
    () -> shooterAimController.getOmegaRadiansPerSecond()
);

// Later, when normal path rotation should resume:
FollowPath.clearRotationOverride();
```

The default override behavior bypasses BLine's rotational velocity and
acceleration constraints so the caller owns the final path-follower omega
command. If the supplied omega should still respect BLine's rotation limits,
use the explicit constrained mode:

```java
FollowPath.overrideRotation(
    () -> shooterAimController.getOmegaRadiansPerSecond(),
    FollowPath.RotationOverrideBehavior.RESPECT_CONSTRAINTS
);
```

### Command-Based Autos With Event Triggers

Register event commands during robot setup using the matching adapter:
`FollowPath.registerEventTrigger("intake", intakeCommand)` for v3, or
`FollowPathV2.registerEventTrigger(...)` for v2. Commands are dispatched on
the scheduler's next event-loop poll and can outlive the path.

Use ordinary coroutine composition for v3. `BLineCommandsV2` retains the
proxy-based v2 composition helpers for autos whose events share mechanism
requirements. Your robot owns cancellation of dispatched events; see the
[cleanup example](WPILIB_2027.md#event-commands).

### Field2d Visualization

BLine paths are polylines, so visualizing them on a WPILib `Field2d` widget (in
Elastic or Glass) needs no simulation. `BLineField` provides small helpers for
drawing a BLine path directly as a connected field object:

```java
import frc.robot.lib.BLine.BLineField;
import org.wpilib.smartdashboard.Field2d;

Field2d field = new Field2d();
// In robotPeriodic(), publish through your WPILib TelemetryTable:
table.log("Field", field);

// Draw a planned path once. BLine assigns a stable unique field object name for
// this path instance and returns it if you want to inspect it.
String objectName = BLineField.drawPath(field, myPath);

// Or choose the display slot yourself. BLine appends "Trajectory" if needed.
BLineField.drawPath(field, "ScoreTwo", myPath);
```

The no-name overload generates names like `"BLinePath0Trajectory"` per
`Field2d`, reusing the same name when called again with the same `Path` instance.
Explicit names are treated as user-owned display slots, so reusing
`"ScoreTwo"` updates `"ScoreTwoTrajectory"`. If you only need the raw polyline
points, use `myPath.getTranslations()`.

## Performance

The original holonomic follower was evaluated with randomized Monte Carlo trials in a WPILib physics
simulation, using Theta* for initial pathfinding and an Artificial Bee Colony
(ABC) optimizer to benchmark the system against PathPlanner.

| Measurement | Result |
| --- | --- |
| Path computation time | **97% reduction** |
| Cross-track error at waypoints | **66% reduction** |
| Total path tracking time | **2.6% decrease** compared to PathPlanner |

These historical results are not validation of the new 2027 tank controller.

Read the **[full white paper](https://docs.google.com/document/d/1Tc87YKWHtsEMEvmVDBD1Ww4e7vIUO2FyK3lwwuf-ZL4/edit?usp=sharing)**.

## Build From Source

```bash
./gradlew build
```

### API Reference

Generate Javadoc locally:

```bash
./gradlew javadoc
# Open build/docs/javadoc/index.html
```

Published API reference: **[Javadoc](https://edanliahovetsky.github.io/BLine-Lib/)**
(stable WPILib 2026 only). Compatibility artifacts retain their own Java 25
Javadoc and sources jars; see the [WPILib 2027 guide](WPILIB_2027.md).

## Troubleshooting

If another robot repo consumes your local `BLine-Lib` checkout (for example via
`includeBuild`) and you run `./gradlew clean` in this repo, rebuild the jar
before the robot repo packages or runs simulation.

```bash
./gradlew jar
```

This regenerates `build/libs/BLine-Lib-<version>.jar` for downstream fat-jar
tasks.

## License

BSD 3-Clause License. See [LICENSE](LICENSE).
