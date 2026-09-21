package frc.robot.lib.BLine;

import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.util.Pair;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Scheduler;

/**
 * Point-to-point following through WPILib Commands v3.
 *
 * <p>Construct a reusable {@link Builder} with robot wiring, then configure execution choices on
 * each returned command. A command reads a fresh copy of its source path when execution begins.
 * Pose reset is off by default. Forward tank driving is the default; it does not reverse element
 * order. All velocities supplied and returned by the robot callbacks are robot-relative, in m/s
 * and rad/s. Translation/CTE PID inputs use metres; heading PID inputs use radians.
 *
 * <p>The drive requirement prevents simultaneous commands from sharing these PID controllers.
 * Supply a distinct controller set for independently running drivetrains. Controller tuning,
 * including integral limits and period, is retained; accumulated state resets each execution.
 */
public final class FollowPath implements Command {
    private static final PendingEvents EVENTS = createEvents();
    private final Follower follower;

    private static PendingEvents createEvents() {
        PendingEvents events = new PendingEvents();
        Scheduler.getDefault().getDefaultEventLoop().bind(events::dispatch);
        return events;
    }
    private final Set<Mechanism> requirements;
    /**
     * Determines how a rotation override interacts with BLine's normal constraints.
     */
    public enum RotationOverrideBehavior {
        /**
         * The supplied omega replaces the rotation PID output before BLine applies its
         * existing rotational velocity and acceleration limits.
         */
        RESPECT_CONSTRAINTS,

        /**
         * BLine still rate-limits translation, but the supplied omega is restored after
         * path-follower limiting so the caller owns the final rotational command.
         */
        BYPASS_CONSTRAINTS
    }

    /**
     * Registers a short non-blocking action. Reached events run on the default scheduler's next
     * event-loop poll, outside the follower. Re-registering a key affects future queued events.
     * @param key event key in authored paths
     * @param action callback; must not block the robot loop
     */
    public static void registerEventTrigger(String key, Runnable action) { EVENTS.register(key, action); }

    /**
     * Registers a command scheduled independently of FollowPath. Completion does not cancel it;
     * the robot application owns cleanup of already scheduled event commands.
     * @param key event key in authored paths
     * @param command command to schedule on the default scheduler
     */
    public static void registerEventTrigger(String key, Command command) {
        Objects.requireNonNull(command, "eventCommand");
        EVENTS.register(key, () -> Scheduler.getDefault().schedule(command));
    }

    /** Clears queued events for this framework without cancelling already scheduled commands. */
    public static void clearPendingEventTriggers() { EVENTS.clear(); }

    /**
     * Overrides the rotational output of all {@code FollowPath} commands.
     *
     * <p>The supplier is called every execution cycle while active and must return omega in
     * radians per second. This overload bypasses BLine's rotational velocity and acceleration
     * limits so the caller owns the final path-follower omega command.
     *
     * <p>Call {@link #clearRotationOverride()} when the override should stop affecting
     * path following.
     *
     * @param supplier supplies omega in radians per second
     * @throws IllegalArgumentException if supplier is null
     */
    public static void overrideRotation(DoubleSupplier supplier) {
        overrideRotation(supplier, RotationOverrideBehavior.BYPASS_CONSTRAINTS);
    }

    /**
     * Overrides the rotational output of all {@code FollowPath} commands.
     *
     * <p>The supplier is called every execution cycle while active and must return omega in
     * radians per second. Use {@link RotationOverrideBehavior#RESPECT_CONSTRAINTS}
     * to keep BLine's normal rotational limits, or
     * {@link RotationOverrideBehavior#BYPASS_CONSTRAINTS} when the caller owns the
     * final rotational command.
     *
     * <p>Call {@link #clearRotationOverride()} when the override should stop affecting
     * path following.
     *
     * @param supplier supplies omega in radians per second
     * @param behavior whether the supplied omega should respect or bypass BLine constraints
     * @throws IllegalArgumentException if supplier or behavior is null
     */
    public static void overrideRotation(DoubleSupplier supplier, RotationOverrideBehavior behavior) {
        if (behavior == null) {
            throw new IllegalArgumentException("Rotation override behavior must not be null");
        }
        Follower.overrideRotation(supplier, Follower.RotationOverrideBehavior.valueOf(behavior.name()));
    }

    /**
     * Clears the active rotation override, restoring normal path rotation control.
     */
    public static void clearRotationOverride() {
        Follower.clearRotationOverride();
    }

    /**
     * Sets the consumer for logging pose data during path following.
     * 
     * <p>The consumer receives pairs of (key, Pose2d) for various internal poses such as
     * closest points on path segments.
     * 
     * @param consumer The consumer to receive pose logging data, or null to disable
     */
    public static void setPoseLoggingConsumer(Consumer<Pair<String, Pose2d>> consumer) {
        Follower.setPoseLoggingConsumer(consumer);
    }

    /**
     * Sets the consumer for logging translation arrays during path following.
     * 
     * <p>The consumer receives pairs of (key, Translation2d[]) for data such as path waypoints
     * and robot position history.
     * 
     * @param consumer The consumer to receive translation list data, or null to disable
     */
    public static void setTranslationListLoggingConsumer(Consumer<Pair<String, Translation2d[]>> consumer) {
        Follower.setTranslationListLoggingConsumer(consumer);
    }

    /**
     * Sets the consumer for logging boolean values during path following.
     * 
     * <p>The consumer receives pairs of (key, Boolean) for state flags such as completion status.
     * 
     * @param consumer The consumer to receive boolean logging data, or null to disable
     */
    public static void setBooleanLoggingConsumer(Consumer<Pair<String, Boolean>> consumer) {
        Follower.setBooleanLoggingConsumer(consumer);
    }

    /**
     * Sets the consumer for logging numeric values during path following.
     * 
     * <p>The consumer receives pairs of (key, Double) for various metrics such as remaining
     * distance, controller outputs, and target indices.
     * 
     * @param consumer The consumer to receive double logging data, or null to disable
     */
    public static void setDoubleLoggingConsumer(Consumer<Pair<String, Double>> consumer) {
        Follower.setDoubleLoggingConsumer(consumer);
    }

    static void setTimestampSupplier(Supplier<Double> supplier) {
        Follower.setTimestampSupplier(supplier);
    }


    /** Stable robot configuration, shared by the independently configured commands it builds. */
    public static final class Builder {
        private final Mechanism drive;
        private final FollowerConfig config;
        private BooleanSupplier shouldFlip;
        private org.wpilib.telemetry.TelemetryTable telemetry;

        /**
         * @param driveType drivetrain model required by this robot
         * @param drive requirement claimed while following
         * @param poseSupplier measured field-relative pose
         * @param resetPose odometry reset callback, used only by commands with pose reset enabled
         * @param measuredVelocity measured robot-relative velocity, not the last requested output
         * @param output robot-relative velocity command (m/s, m/s, rad/s)
         * @param translation translation distance PID (metres to m/s)
         * @param rotation continuous heading PID (radians to rad/s)
         * @param crossTrack cross-track distance PID (metres to m/s)
         */
        public Builder(DriveType driveType, Mechanism drive, Supplier<Pose2d> poseSupplier,
            Consumer<Pose2d> resetPose, Supplier<ChassisVelocities> measuredVelocity,
            Consumer<ChassisVelocities> output, PIDController translation,
            PIDController rotation, PIDController crossTrack) {
            this.drive = Objects.requireNonNull(drive, "drive");
            config = new FollowerConfig(driveType, poseSupplier, resetPose, measuredVelocity,
                output, translation, rotation, crossTrack);
        }

        /**
         * Sets the policy copied into subsequently built commands; it is sampled once per run.
         * @param supplier true for flipped, false for unflipped
         * @return this builder
         */
        public Builder withShouldFlip(BooleanSupplier supplier) {
            shouldFlip = Objects.requireNonNull(supplier, "shouldFlip");
            return this;
        }

        /**
         * Logs the existing BLine signals under this table in subsequently built commands.
         * Legacy logging callbacks remain available and still receive the same values.
         * @param table application-owned WPILib telemetry table, or null to disable
         * @return this builder
         */
        public Builder withTelemetry(org.wpilib.telemetry.TelemetryTable table) {
            telemetry = table;
            return this;
        }

        /** @return this builder, configured to flip for the red alliance (unknown means unflipped) */
        public Builder withDefaultShouldFlip() {
            return withShouldFlip(FollowerConfig::redAlliance);
        }

        /**
         * Creates a command without enabling pose reset. Without a flip policy, the source path's
         * transform state is preserved. Later changes to this builder do not affect this command.
         * @param path mutable authored path, snapshotted at each execution
         * @return a new independently configurable command
         */
        public FollowPath build(Path path) {
            return new FollowPath(drive, new Follower(path, config, shouldFlip, EVENTS).withTelemetry(telemetry));
        }
    }

    private FollowPath(Mechanism drive, Follower follower) {
        this.follower = follower;
        requirements = Set.of(drive);
    }

    /**
     * Enables reset to an authored start on every execution. A path without an authored start
     * reports one warning and skips reset; its execution origin is the measured pose instead.
     * @return this command
     */
    public FollowPath withPoseReset() { follower.withPoseReset(); return this; }

    /**
     * Overrides only this command's flip policy, evaluated once per execution.
     * @param supplier desired flipped state
     * @return this command
     */
    public FollowPath withShouldFlip(BooleanSupplier supplier) { follower.withShouldFlip(supplier); return this; }

    /**
     * Sets only this command's reflection policy, evaluated once per execution.
     * @param supplier desired mirrored state
     * @return this command
     */
    public FollowPath withShouldMirror(BooleanSupplier supplier) { follower.withShouldMirror(supplier); return this; }

    /**
     * Selects which end of a tank robot leads along the path. Backward is invalid for holonomic
     * drive types and is rejected before pose reset or events.
     * @param direction forward or backward travel
     * @return this command
     */
    public FollowPath withTankDriveDirection(DriveDirection direction) {
        follower.withTankDriveDirection(direction); return this;
    }

    /** @return current expanded rotation element index, or -1 without an active target */
    public int getCurrentRotationElementIndex() { return follower.getCurrentRotationElementIndex(); }

    /** @return current expanded translation element index */
    public int getCurrentTranslationElementIndex() { return follower.getCurrentTranslationElementIndex(); }

    /** @return remaining translational distance in metres, or zero before initialization */
    public double getRemainingPathDistanceMeters() { return follower.getRemainingPathDistanceMeters(); }


    @Override public String name() { return "FollowPath"; }
    @Override public Set<Mechanism> requirements() { return requirements; }

    @Override
    public void run(Coroutine co) {
        try {
            follower.initialize();
            do {
                follower.execute();
                if (follower.isFinished()) break;
                co.yield();
            } while (true);
            follower.end(false);
        } catch (RuntimeException | Error error) {
            follower.end(true);
            throw error;
        }
    }

    @Override public void onCancel() { follower.end(true); }
}
