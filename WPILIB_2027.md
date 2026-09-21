# BLine-Lib 2027 beta

This development candidate targets WPILib `2027.0.0-alpha-7` and Java 25.
It is pending manual review and distribution. The stable WPILib 2026 release
remains on [`main`](https://github.com/edanliahovetsky/BLine-Lib).

## Command interfaces

Use `FollowPath.Builder` with a Commands v3 `Mechanism`, or
`FollowPathV2.Builder` with a Commands v2 `Subsystem`. Both accept the robot's
`DriveType`, pose supplier, pose-reset callback, measured robot-relative
velocity supplier, robot-relative output callback, and translation, rotation,
and cross-track `PIDController` instances, in that order.

The adapters share the same follower behavior and options. Install the command
framework your robot uses; BLine compiles against both without adding either
framework as a runtime dependency. WPILib 2027 imports use `org.wpilib`.

## Tank driving direction

The editor's **Drive forward** / **Drive backward** arrow buttons save the path's
robot driving direction. Preview and generation use that same direction.
Exported path JSON includes:

```json
"tank_drive_direction": "backward"
```

Older paths without the property default to Forward. The editor migrates an
older `preview.tank_direction` setting when the new property is absent, removing
that duplicate metadata value. The ghost starting pose remains private editor
metadata.

With either command builder:

```java
builder.build(path); // Use the direction saved in the path.
builder.build(path).withTankDriveDirection(DriveDirection.FORWARD);
builder.build(path).withTankDriveDirection(DriveDirection.BACKWARD);
```

For programmatic paths:

```java
path.setTankDriveDirection(DriveDirection.BACKWARD);
DriveDirection direction = path.getTankDriveDirection();
```

Each execution snapshots the path and resolves the command override first,
then the saved direction. Reusing a command without an override picks up source
`Path` changes on the next execution. Files are not reread automatically, and
an active run keeps its direction. Overrides never mutate the path, builder,
or another command.

Backward means rear-first travel through the same element order. Authored
headings, transforms, and pose-reset behavior are unchanged. Swerve and mecanum
ignore the saved tank-specific property; explicitly requesting Backward on a
holonomic command is an invalid configuration.
