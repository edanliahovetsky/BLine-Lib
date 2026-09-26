<h1 align="center">BLine-Lib</h1>

<p align="center">
  <a href="BLine-Lib.json"><img src="https://img.shields.io/badge/version-v0.9.1-2563eb" alt="Version v0.9.1"></a>
  <a href="BLine-Lib.json"><img src="https://img.shields.io/badge/FRC-2026-c1121f" alt="FRC 2026"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-BSD--3--Clause-0f766e" alt="BSD 3-Clause License"></a>
</p>

**BLine** is a rapid point-to-point autonomous path planning and tracking
library for FIRST Robotics Competition. It is made by students for students and
built around practical tuning, quick iteration, and rapid empirical testing in
time-constrained build-season environments.

**Quick links**

🚀 **[Open the hosted editor](https://bline-web.pages.dev/)** — create, tune, preview, and export BLine paths in the browser.

🖥️ **[BLine Web](https://github.com/edanliahovetsky/BLine-Web)** — current web and desktop editor.

💬 **[Chief Delphi Thread](https://www.chiefdelphi.com/t/introducing-bline-a-new-rapid-polyline-autonomous-path-planning-suite/509778)** — discussion, feedback, and announcements.

📚 **[Documentation](https://bline-docs.pages.dev/)** — full guides, tutorials, and reference.

## Supported release lines

`main` remains the stable, default BLine-Lib line for WPILib 2026, and the
installation guidance below is for that line. Teams evaluating WPILib 2027
Alpha 6 can use the behavior-compatible [`wpilib-2027` line and its separate
installation guide](https://github.com/edanliahovetsky/BLine-Lib/blob/wpilib-2027/WPILIB_2027.md).

<p align="center">
  <img src="docs/readme/bline-web-demo.gif" alt="BLine Web editor GUI demo" width="900">
  <br><br>
  <img src="docs/cone-demo.gif" alt="BLine robot cone demo" width="900">
</p>

## Installation

### Vendor JSON (Recommended)

BLine-Lib's recommended WPILib vendor URL is the BLine Metrics Worker endpoint.
It serves the same vendor JSON as this repository while keeping aggregate fetch
counts for release health.

1. Open VS Code with your FRC project
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
3. Type **"WPILib: Manage Vendor Libraries"**
4. Select **"Install new libraries (online)"**
5. Paste the recommended vendor URL:

```text
https://bline-metrics.edan-liahovetsky.workers.dev/vendor/BLine-Lib.json
```

If the Worker URL is temporarily unavailable, use the direct GitHub vendor JSON
as the fallback:

```text
https://raw.githubusercontent.com/edanliahovetsky/BLine-Lib/main/BLine-Lib.json
```

### Gradle Dependency (Alternative)

Add the JitPack repository to your `build.gradle`:

```gradle
repositories {
    maven { url 'https://jitpack.io' }
}
```

Then add the BLine-Lib dependency:

```gradle
dependencies {
    implementation 'com.github.edanliahovetsky:BLine-Lib:v0.9.1'
}
```

## Quick Start

For a complete walkthrough, see the **[getting started guide](https://bline-docs.pages.dev/getting-started/)**.

### Basic Setup

```java
import frc.robot.lib.BLine.*;
import edu.wpi.first.math.controller.PIDController;

// 1. Set global constraints
Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
    4.0,    // maxVelocityMetersPerSec
    3.0,    // maxAccelerationMetersPerSec2
    360.0,  // maxVelocityDegPerSec
    720.0,  // maxAccelerationDegPerSec2
    0.05,   // endTranslationToleranceMeters
    2.0,    // endRotationToleranceDeg
    0.3     // intermediateHandoffRadiusMeters
));

// 2. Create a reusable path builder
FollowPath.Builder pathBuilder = new FollowPath.Builder(
    driveSubsystem,
    driveSubsystem::getPose,
    driveSubsystem::getChassisSpeeds,
    driveSubsystem::drive,
    new PIDController(5.0, 0.0, 0.0),  // translation
    new PIDController(3.0, 0.0, 0.0),  // rotation
    new PIDController(2.0, 0.0, 0.0)   // cross-track
).withDefaultShouldFlip()
 .withPoseReset(driveSubsystem::resetPose);

// 3. Load and follow a path
Path myPath = new Path("myPathFile");  // loads deploy/autos/paths/myPathFile.json
Command followCommand = pathBuilder.build(myPath);
```

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

When a BLine path contains event triggers that schedule WPILib commands, prefer
`BLineCommands` for the surrounding command composition:

```java
import static frc.robot.lib.BLine.BLineCommands.sequence;
import edu.wpi.first.wpilibj2.command.Command;

Command auto = sequence(
    shooter.shoot().withTimeout(2.0),
    pathing.followPath("intakethroughdepot"),
    shooter.shoot()
);
```

`BLineCommands` mirrors the WPILib `Commands` composition methods that accept
child commands, but proxies those children before building the group. This keeps
the outer auto from owning every child requirement for its whole lifetime, which
lets BLine event-trigger commands use normal WPILib scheduling. See the
`BLineCommands` Javadocs for method-by-method behavior and limitations. The API
intentionally contains only WPILib `Commands` counterparts: `either`, `select`,
`defer`, `deferredProxy`, `sequence`, `repeatingSequence`, `parallel`, `race`,
and `deadline`.

### Field2d Visualization

BLine paths are polylines, so visualizing them on a WPILib `Field2d` widget (in
Elastic or Glass) needs no simulation. `BLineField` provides small helpers for
drawing a BLine path directly as a connected field object:

```java
import frc.robot.lib.BLine.BLineField;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

Field2d field = new Field2d();
SmartDashboard.putData("Field", field);

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

BLine has been validated with randomized Monte Carlo trials in a WPILib physics
simulation, using Theta* for initial pathfinding and an Artificial Bee Colony
(ABC) optimizer to benchmark the system against PathPlanner.

| Measurement | Result |
| --- | --- |
| Path computation time | **97% reduction** |
| Cross-track error at waypoints | **66% reduction** |
| Total path tracking time | **2.6% decrease** compared to PathPlanner |

Read the **[full white paper](https://docs.google.com/document/d/1Tc87YKWHtsEMEvmVDBD1Ww4e7vIUO2FyK3lwwuf-ZL4/edit?usp=sharing)**.

## Build From Source

See [Contributing](CONTRIBUTING.md) for stable and beta branches, release
promotion, and local worktrees.

```bash
./gradlew build
```

### API Reference

Generate Javadoc locally:

```bash
./gradlew javadoc
# Open build/docs/javadoc/index.html
```

Published API reference: **[Javadoc](https://edanliahovetsky.github.io/BLine-Lib/)**.

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
