# BLine-Lib

A path following library for FRC robots.

## Installation

### Using JitPack (Recommended for FRC)

Add JitPack repository to your `build.gradle`:

```gradle
repositories {
    maven { url 'https://jitpack.io' }
}
```

Add the dependency:

```gradle
dependencies {
    implementation 'com.github.YOUR_GITHUB_USERNAME:BLine-Lib:VERSION'
}
```

Replace `YOUR_GITHUB_USERNAME` with your actual GitHub username and `VERSION` with a release tag (e.g., `v1.0.0`), commit hash, or `main-SNAPSHOT` for the latest main branch.

### Using Vendor JSON (FRC Standard Method)

You can also install this as a vendor library. Add the following URL to your VSCode "WPILib: Manage Vendor Libraries" → "Install new library (online)":

```
https://raw.githubusercontent.com/YOUR_GITHUB_USERNAME/BLine-Lib/main/BLine.json
```

## Quick Start

### 1. Configure the Path Follower

In your `RobotContainer` or subsystem initialization:

```java
import frc.robot.lib.BLine.*;
import edu.wpi.first.math.controller.PIDController;

// Set up PID controllers for translation and rotation
PIDController translationPID = new PIDController(5.0, 0.0, 0.0);
PIDController rotationPID = new PIDController(3.0, 0.0, 0.0);
PIDController crossTrackPID = new PIDController(1.0, 0.0, 0.0);

FollowPath.setTranslationController(translationPID);
FollowPath.setRotationController(rotationPID);
FollowPath.setCrossTrackController(crossTrackPID);
```

### 2. Create and Follow a Path

```java
// Load a path from file (from deploy/autos/paths/)
Path myPath = new Path("myPathFile");

// Or create a path programmatically
Path myPath = new Path(
    new Path.Waypoint(new Translation2d(1.0, 1.0), new Rotation2d(0)),
    new Path.TranslationTarget(new Translation2d(2.0, 2.0)),
    new Path.Waypoint(new Translation2d(3.0, 1.0), new Rotation2d(Math.PI))
);

// Create the follow command
Command followCommand = new FollowPath(
    myPath,
    driveSubsystem,
    () -> driveSubsystem.getPose(),
    pose -> driveSubsystem.resetPose(pose),
    () -> shouldFlipForRedAlliance(),
    () -> driveSubsystem.getChassisSpeeds(),
    speeds -> driveSubsystem.drive(speeds)
);
```

### 3. Set Global Constraints

Create a `config.json` file in `src/main/deploy/autos/`:

```json
{
    "default_max_velocity_meters_per_sec": 4.0,
    "default_max_acceleration_meters_per_sec2": 3.0,
    "default_max_velocity_deg_per_sec": 360.0,
    "default_max_acceleration_deg_per_sec2": 720.0,
    "default_end_translation_tolerance_meters": 0.05,
    "default_end_rotation_tolerance_deg": 2.0,
    "default_intermediate_handoff_radius_meters": 0.3
}
```

## Path File Format

Paths are stored as JSON files in `deploy/autos/paths/`. Example:

```json
{
    "path_elements": [
        {
            "type": "waypoint",
            "translation_target": {
                "x_meters": 1.0,
                "y_meters": 1.0,
                "intermediate_handoff_radius_meters": 0.3
            },
            "rotation_target": {
                "rotation_radians": 0,
                "t_ratio": 1.0,
                "profiled_rotation": true
            }
        },
        {
            "type": "translation",
            "x_meters": 2.5,
            "y_meters": 2.0,
            "intermediate_handoff_radius_meters": 0.3
        },
        {
            "type": "rotation",
            "rotation_radians": 1.57,
            "t_ratio": 0.5,
            "profiled_rotation": true
        }
    ]
}
```

## Building from Source

```bash
./gradlew build
```

## License

BSD 3-Clause License — See [LICENSE](LICENSE) file.
