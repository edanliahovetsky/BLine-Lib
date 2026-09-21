package frc.robot.lib.BLine;

import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Stable robot wiring. Execution choices and traversal state belong to each follower. */
record FollowerConfig(
    DriveType driveType,
    Supplier<Pose2d> pose,
    Consumer<Pose2d> resetPose,
    Supplier<ChassisVelocities> measuredVelocity,
    Consumer<ChassisVelocities> output,
    PIDController translation,
    PIDController rotation,
    PIDController crossTrack
) {
    FollowerConfig {
        Objects.requireNonNull(driveType, "driveType");
        Objects.requireNonNull(pose, "poseSupplier");
        Objects.requireNonNull(resetPose, "resetPose");
        Objects.requireNonNull(measuredVelocity, "measuredRobotRelativeVelocity");
        Objects.requireNonNull(output, "robotRelativeOutput");
        Objects.requireNonNull(translation, "translationController");
        Objects.requireNonNull(rotation, "rotationController");
        Objects.requireNonNull(crossTrack, "crossTrackController");
        if (translation == rotation || translation == crossTrack || rotation == crossTrack) {
            throw new IllegalArgumentException("Translation, rotation and cross-track need separate PID controllers");
        }
    }

    static boolean redAlliance() {
        return org.wpilib.driverstation.MatchState.getAlliance()
            .filter(a -> a == org.wpilib.driverstation.Alliance.RED).isPresent();
    }
}
