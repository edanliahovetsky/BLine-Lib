package frc.robot.lib.BLine;

import org.wpilib.util.Pair;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Subsystem;
import frc.robot.lib.BLine.Path.EventTrigger;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * A WPILib Command that follows a {@link Path} using PID controllers for translation and rotation.
 * 
 * <p>This command drives the robot along a defined path by tracking translation targets sequentially
 * while simultaneously managing rotation targets. The command uses three PID controllers:
 * <ul>
 *   <li><b>Translation Controller:</b> Calculates command speed by minimizing total path distance remaining</li>
 *   <li><b>Rotation Controller:</b> Controls holonomic rotation toward the current rotation target</li>
 *   <li><b>Cross-Track Controller:</b> Minimizes deviation from the line between waypoints</li>
 * </ul>
 * 
 * <p>The path following algorithm works by:
 * <ol>
 *   <li>Calculating command robot speed via a PID controller minimizing total path distance remaining</li>
 *   <li>Determining velocity direction by pointing toward the current translation target</li>
 *   <li>Advancing to the next translation target when within the handoff radius of the current one</li>
 *   <li>Applying cross-track correction to stay on the line between waypoints</li>
 *   <li>Interpolating rotation based on progress between rotation targets</li>
 *   <li>Applying rate limiting via {@link ChassisRateLimiter} to respect constraints</li>
 * </ol>
 * 
 * <h2>Usage</h2>
 * <p>Use the {@link Builder} class to construct FollowPath commands:
 * <pre>{@code
 * FollowPath.Builder pathBuilder = new FollowPath.Builder(
 *     driveSubsystem,
 *     this::getPose,
 *     this::getRobotRelativeSpeeds,
 *     this::driveRobotRelative,
 *     new PIDController(5.0, 0, 0),  // translation
 *     new PIDController(3.0, 0, 0),  // rotation
 *     new PIDController(2.0, 0, 0)   // cross-track
 * ).withDefaultShouldFlip()
 *  .withPoseReset(this::resetPose);
 * 
 * // Then build commands for specific paths:
 * Command followAuto = pathBuilder.build(new Path("myPath"));
 * }</pre>
 * 
 * <h2>Logging</h2>
 * <p>The command supports optional logging via consumer functions. Set up logging callbacks using:
 * <ul>
 *   <li>{@link #setPoseLoggingConsumer(Consumer)} - Log pose data</li>
 *   <li>{@link #setDoubleLoggingConsumer(Consumer)} - Log numeric values</li>
 *   <li>{@link #setBooleanLoggingConsumer(Consumer)} - Log boolean states</li>
 *   <li>{@link #setTranslationListLoggingConsumer(Consumer)} - Log translation arrays</li>
 * </ul>
 * 
 * @see Path
 * @see Builder
 * @see ChassisRateLimiter
 */
public class FollowPath extends Command {
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

    private final Follower follower;

    /**
     * Registers an event trigger action by key.
     *
     * <p>The {@code key} must match the {@code lib_key} of an {@link EventTrigger}
     * in a path JSON file. The action runs inline from this {@code FollowPath}
     * command when the trigger's {@code t_ratio} is reached, so it should be quick
     * and non-blocking. Use {@link #registerEventTrigger(String, Command)} when the
     * trigger should start a normal WPILib command.
     *
     * @param key The event trigger key referenced in JSON
     * @param action The action to execute when the trigger is reached
     */
    public static void registerEventTrigger(String key, Runnable action) {
        Follower.registerEventTrigger(key, action);
    }

    /**
     * Registers an event trigger action by key using a WPILib Command.
     *
     * <p>The {@code key} must match the {@code lib_key} of an {@link EventTrigger}
     * in a path JSON file. When the marker is reached, BLine schedules the supplied
     * command with WPILib's {@link CommandScheduler}. The command is not automatically
     * proxied here and is not automatically canceled when the path ends; it runs until
     * it finishes or is interrupted by normal WPILib scheduling rules.
     *
     * <p>If this event command requires a subsystem that is also used by commands before
     * or after the path, compose the surrounding autonomous routine with
     * {@link BLineCommands} so the outer composition does not hold those child
     * requirements for its whole lifetime.
     *
     * @param key The event trigger key referenced in JSON
     * @param command The command to schedule when the trigger is reached
     */
    public static void registerEventTrigger(String key, Command command) {
        if (command == null) {
            Follower.registerEventTrigger(key, null);
            return;
        }
        registerEventTrigger(key, () -> CommandScheduler.getInstance().schedule(command));
    }

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

    /**
     * Builder class for constructing {@link FollowPath} commands with a fluent API.
     * 
     * <p>The Builder allows you to configure a path follower once with all the robot-specific
     * parameters, then build multiple commands for different paths. This avoids repeating
     * the same configuration for each path.
     *
     * <p><b>Important:</b> This builder is mutable and stateful. Optional settings configured
     * through {@code with...} methods persist for all subsequent {@link #build(Path)} calls
     * until you change them again. For example, once {@link #withPoseReset(Consumer)} is set,
     * later built commands will continue resetting pose unless you override it (for example,
     * with a no-op consumer).
     * 
     * <h2>Required Parameters</h2>
     * <p>The constructor requires:
     * <ul>
     *   <li>Drive subsystem - For command requirements</li>
     *   <li>Pose supplier - Returns current robot pose</li>
     *   <li>Robot-relative speeds supplier - Returns current chassis speeds</li>
     *   <li>Robot-relative speeds consumer - Accepts commanded chassis speeds</li>
     *   <li>Three PID controllers for translation, rotation, and cross-track correction</li>
     * </ul>
     * 
     * <h2>Optional Configuration</h2>
     * <ul>
     *   <li>{@link #withShouldFlip(Supplier)} - Custom alliance flip logic</li>
     *   <li>{@link #withDefaultShouldFlip()} - Use DriverStation alliance for flipping</li>
     *   <li>{@link #withShouldMirror(Supplier)} - Custom vertical mirror logic</li>
     *   <li>{@link #withPoseReset(Consumer)} - Reset odometry to path start pose</li>
     * </ul>
     * 
     * <h2>Example</h2>
     * <pre>{@code
     * FollowPath.Builder builder = new FollowPath.Builder(
     *     driveSubsystem,
     *     this::getPose,
     *     this::getSpeeds,
     *     this::drive,
     *     translationPID,
     *     rotationPID,
     *     crossTrackPID
     * ).withDefaultShouldFlip();
     * 
     * Command cmd = builder.build(myPath);
     * }</pre>
     */
    public static class Builder {
        private final Subsystem driveSubsystem;
        private final Supplier<Pose2d> poseSupplier;
        private final Supplier<ChassisVelocities> robotRelativeSpeedsSupplier;
        private final Consumer<ChassisVelocities> robotRelativeSpeedsConsumer;
        private final PIDController translationController;
        private final PIDController rotationController;
        private final PIDController crossTrackController;
        
        private Supplier<Boolean> shouldFlipPathSupplier = () -> false;
        private Supplier<Boolean> shouldMirrorPathSupplier = () -> false;
        private Consumer<Pose2d> poseResetConsumer = (pose) -> {};
        private boolean useTRatioBasedTranslationHandoffs = false;
        
        /**
         * Creates a new FollowPath Builder with the required configuration.
         * 
         * @param driveSubsystem The drive subsystem that the command will require
         * @param poseSupplier Supplier that returns the current robot pose in field coordinates
         * @param robotRelativeSpeedsSupplier Supplier that returns current robot-relative chassis speeds
         * @param robotRelativeSpeedsConsumer Consumer that accepts robot-relative chassis speeds to drive
         * @param translationController PID controller for calculating command speed by minimizing path distance remaining
         * @param rotationController PID controller for rotating toward rotation targets
         * @param crossTrackController PID controller for staying on the line between waypoints
         */
        public Builder(
            Subsystem driveSubsystem, 
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisVelocities> robotRelativeSpeedsSupplier,
            Consumer<ChassisVelocities> robotRelativeSpeedsConsumer,
            PIDController translationController,
            PIDController rotationController,
            PIDController crossTrackController
        ) {
            this.driveSubsystem = driveSubsystem;
            this.poseSupplier = poseSupplier;
            this.robotRelativeSpeedsSupplier = robotRelativeSpeedsSupplier;
            this.robotRelativeSpeedsConsumer = robotRelativeSpeedsConsumer;
            this.translationController = translationController;
            this.rotationController = rotationController;
            this.crossTrackController = crossTrackController;
        }

        /**
         * Configures a custom supplier to determine whether the path should be flipped.
         * 
         * <p>When the supplier returns true, the path will be flipped to the opposite alliance
         * side using {@link FlippingUtil} during command initialization.
         *
         * <p>This setting persists for future {@link #build(Path)} calls until changed.
         * 
         * @param shouldFlipPathSupplier Supplier returning true if the path should be flipped
         * @return This builder for chaining
         */
        public Builder withShouldFlip(Supplier<Boolean> shouldFlipPathSupplier) {
            this.shouldFlipPathSupplier = shouldFlipPathSupplier;
            return this;
        }

        /**
         * Configures the builder to use the default alliance-based path flipping.
         * 
         * <p>When enabled, paths will automatically be flipped when the robot is on the
         * red alliance, based on {@link org.wpilib.driverstation.MatchState#getAlliance()}.
         *
         * <p>This setting persists for future {@link #build(Path)} calls until changed.
         * 
         * @return This builder for chaining
         */
        public Builder withDefaultShouldFlip() {
            this.shouldFlipPathSupplier = FollowPath::shouldFlipPath;
            return this;
        }

        /**
         * Configures a custom supplier to determine whether the path should be mirrored vertically.
         * 
         * <p>When the supplier returns true, the path will be mirrored over the vertical
         * direction across the field width ({@code y -> fieldSizeY - y}) via {@link Path#mirror()}.
         *
         * <p>This setting persists for future {@link #build(Path)} calls until changed.
         * 
         * @param shouldMirrorPathSupplier Supplier returning true if the path should be mirrored vertically
         * @return This builder for chaining
         */
        public Builder withShouldMirror(Supplier<Boolean> shouldMirrorPathSupplier) {
            this.shouldMirrorPathSupplier = shouldMirrorPathSupplier;
            return this;
        }

        /**
         * Configures a consumer to reset the robot's pose at the start of path following.
         * 
         * <p>When set, the command will call this consumer with the path's starting pose
         * during initialization. This is useful for resetting odometry when starting autonomous
         * routines or when the robot is placed at a known location.
         *
         * <p>This setting persists for future {@link #build(Path)} calls until changed.
         * To disable pose reset on later commands when reusing the same builder, set a no-op
         * consumer such as {@code withPoseReset(pose -> {})}.
         * 
         * @param poseResetConsumer Consumer that resets the robot's pose estimate
         * @return This builder for chaining
         */
        public Builder withPoseReset(Consumer<Pose2d> poseResetConsumer) {
            this.poseResetConsumer = poseResetConsumer;
            return this;
        }

        /**
         * Enables or disables t_ratio-based translation handoffs.
         *
         * <p>When enabled, translation target handoffs occur based on projected
         * segment progress rather than raw distance, which can be more robust at
         * higher speeds on riskier paths. Defaults to false.
         *
         * <p>This setting persists for future {@link #build(Path)} calls until changed.
         *
         * @param enabled true to use t_ratio-based handoffs, false for radius-based
         * @return This builder for chaining
         */
        public Builder withTRatioBasedTranslationHandoffs(boolean enabled) {
            this.useTRatioBasedTranslationHandoffs = enabled;
            return this;
        }

        /**
         * Builds a FollowPath command for the specified path.
         * 
         * <p>The built command will use all the configuration from this builder. Each call
         * to build() creates an independent command that can be scheduled, using the builder's
         * current optional settings at the time of the call.
         * 
         * @param path The path to follow
         * @return A new FollowPath command configured for the given path
         * @throws IllegalArgumentException if any required controllers are null
         */
        public FollowPath build(Path path) {
            return new FollowPath(
                path,
                driveSubsystem,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                robotRelativeSpeedsConsumer,
                shouldFlipPathSupplier,
                shouldMirrorPathSupplier,
                poseResetConsumer,
                useTRatioBasedTranslationHandoffs,
                translationController,
                rotationController,
                crossTrackController
            );
        }
    }

    /**
     * Determines if the path should be flipped based on the current alliance.
     * 
     * @return true if on the red alliance and the path should be flipped, false otherwise
     */
    private static boolean shouldFlipPath() {
        var alliance = org.wpilib.driverstation.MatchState.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == org.wpilib.driverstation.Alliance.RED;
        }
        return false;
    }

    private FollowPath(
        Path path,
        Subsystem driveSubsystem,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisVelocities> robotRelativeSpeedsSupplier,
        Consumer<ChassisVelocities> robotRelativeSpeedsConsumer,
        Supplier<Boolean> shouldFlipPathSupplier,
        Supplier<Boolean> shouldMirrorPathSupplier,
        Consumer<Pose2d> poseResetConsumer,
        boolean useTRatioBasedTranslationHandoffs,
        PIDController translationController,
        PIDController rotationController,
        PIDController crossTrackController
    ) {
        follower = new Follower(path, poseSupplier, robotRelativeSpeedsSupplier,
            robotRelativeSpeedsConsumer, shouldFlipPathSupplier, shouldMirrorPathSupplier,
            poseResetConsumer, useTRatioBasedTranslationHandoffs, translationController,
            rotationController, crossTrackController);
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        follower.initialize();
    }

    @Override
    public void execute() {
        follower.execute();
    }

    @Override
    public boolean isFinished() {
        return follower.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        follower.end(interrupted);
    }

    public int getCurrentRotationElementIndex() {
        return follower.getCurrentRotationElementIndex();
    }

    public int getCurrentTranslationElementIndex() {
        return follower.getCurrentTranslationElementIndex();
    }

    public double getRemainingPathDistanceMeters() {
        return follower.getRemainingPathDistanceMeters();
    }
}
