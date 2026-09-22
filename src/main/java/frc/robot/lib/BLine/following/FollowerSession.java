package frc.robot.lib.BLine.following;

import frc.robot.lib.BLine.path.DriveDirection;
import frc.robot.lib.BLine.path.Path;
import java.util.Objects;
import java.util.function.*;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.*;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.util.Pair;

/**
 * Lifecycle bridge used by the two command adapters.
 * <p>Internal library API, public only for access across packages. Robot code should use
 * FollowPath or FollowPathV2; traversal and controllers remain package-private.
 */
public final class FollowerSession {
    /** One execution's reached events. Dispatch and command ownership belong to the adapter. */
    public interface EventRun {
        void enqueue(String key);
        void cancel();
    }
    public enum RotationOverrideBehavior { RESPECT_CONSTRAINTS, BYPASS_CONSTRAINTS }
    public record Config(
        DriveType driveType,
        Supplier<Pose2d> pose,
        Consumer<Pose2d> resetPose,
        Supplier<ChassisVelocities> measuredVelocity,
        Consumer<ChassisVelocities> output,
        PIDController translation,
        PIDController rotation,
        PIDController crossTrack
    ) {
        public Config {
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
    }
    private final Follower follower;
    public FollowerSession(Path path, Config config, BooleanSupplier flip, Supplier<EventRun> events) {
        follower = new Follower(path, config, flip, events);
    }
    public FollowerSession withTelemetry(TelemetryTable table) { follower.withTelemetry(table); return this; }
    public void withPoseReset() { follower.withPoseReset(); }
    public void withTankDriveDirection(DriveDirection direction) { follower.withTankDriveDirection(direction); }
    public void withShouldFlip(BooleanSupplier flip) { follower.withShouldFlip(flip); }
    public void withShouldMirror(BooleanSupplier mirror) { follower.withShouldMirror(mirror); }
    public void initialize() { follower.initialize(); }
    public void execute() { follower.execute(); }
    public boolean isFinished() { return follower.isFinished(); }
    public void end(boolean interrupted) { follower.end(interrupted); }
    public int getCurrentTranslationElementIndex() { return follower.getCurrentTranslationElementIndex(); }
    public int getCurrentRotationElementIndex() { return follower.getCurrentRotationElementIndex(); }
    public double getRemainingPathDistanceMeters() { return follower.getRemainingPathDistanceMeters(); }
    public static boolean redAlliance() {
        return org.wpilib.driverstation.MatchState.getAlliance()
            .filter(a -> a == org.wpilib.driverstation.Alliance.RED).isPresent();
    }
    public static void overrideRotation(DoubleSupplier supplier, RotationOverrideBehavior behavior) {
        Follower.overrideRotation(supplier, Follower.RotationOverrideBehavior.valueOf(behavior.name()));
    }
    public static void clearRotationOverride() { Follower.clearRotationOverride(); }
    public static void setTimestampSupplier(Supplier<Double> clock) { Follower.setTimestampSupplier(clock); }
    public static void setPoseLoggingConsumer(Consumer<Pair<String, Pose2d>> consumer) { Follower.setPoseLoggingConsumer(consumer); }
    public static void setTranslationListLoggingConsumer(Consumer<Pair<String, Translation2d[]>> consumer) { Follower.setTranslationListLoggingConsumer(consumer); }
    public static void setDoubleLoggingConsumer(Consumer<Pair<String, Double>> consumer) { Follower.setDoubleLoggingConsumer(consumer); }
    public static void setBooleanLoggingConsumer(Consumer<Pair<String, Boolean>> consumer) { Follower.setBooleanLoggingConsumer(consumer); }
}
