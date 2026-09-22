package frc.robot.lib.BLine.path;

import frc.robot.lib.BLine.field.FlippingUtil;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.util.Pair;

/**
 * Represents a path that a robot can follow, consisting of translation and rotation targets.
 * 
 * <p>A Path is composed of {@link PathElement} objects that define where the robot should go
 * (translation targets) and what holonomic rotation it should have (rotation targets). These can be
 * combined into waypoints for convenience.
 * 
 * <h2>Path Elements</h2>
 * <ul>
 *   <li>{@link TranslationTarget} - A position on the field the robot should drive through</li>
 *   <li>{@link RotationTarget} - A holonomic rotation the robot should achieve at a certain point along a segment</li>
 *   <li>{@link Waypoint} - A combined translation and rotation target</li>
 * </ul>
 * 
 * <h2>Constraints</h2>
 * <p>Paths support both path-specific constraints ({@link PathConstraints}) and global default
 * constraints ({@link DefaultGlobalConstraints}). Path-specific constraints override global
 * defaults when specified.
 * 
 * <h2>Creating Paths</h2>
 * <p>Paths can be created programmatically or loaded from JSON files:
 * <pre>{@code
 * // Programmatic path creation
 * Path path = new Path(
 *     new Waypoint(new Pose2d(1, 1, Rotation2d.fromDegrees(0))),
 *     new TranslationTarget(2, 2),
 *     new RotationTarget(Rotation2d.fromDegrees(90), 0.5),
 *     new Waypoint(new Pose2d(3, 1, Rotation2d.fromDegrees(180)))
 * );
 * 
 * // Load from JSON file
 * Path path = new Path("myPath");  // loads from deploy/autos/paths/myPath.json
 * }</pre>
 * 
 * <h2>Alliance Flipping</h2>
 * <p>Paths can be flipped to the opposite alliance side using {@link #flip()} and
 * {@link #undoFlip()}. This uses {@link FlippingUtil} to transform coordinates.
 * 
 * @see PathElement
 * @see PathConstraints
 * @see DefaultGlobalConstraints
 * @see JsonPathCodec
 */
public class Path {
    private static final Logger logger = Logger.getLogger(Path.class.getName());

    /**
     * Sealed interface for all path element types.
     * 
     * <p>Path elements define the targets that make up a path. The three permitted
     * implementations are:
     * <ul>
     *   <li>{@link Waypoint} - Combined translation and rotation target</li>
     *   <li>{@link TranslationTarget} - Position target only</li>
     *   <li>{@link RotationTarget} - Holonomic rotation target only</li>
     * </ul>
     */
    public sealed interface PathElement permits Waypoint, TranslationTarget, RotationTarget, EventTrigger {
        /**
         * Creates a deep copy of this path element.
         * 
         * @return A new PathElement with the same values
         */
        public PathElement copy();
    }

    /**
     * Sealed interface for constraints associated with path elements.
     * 
     * <p>Each path element type has a corresponding constraint type that defines
     * the velocity and acceleration limits for that element.
     */
    sealed interface PathElementConstraint permits WaypointConstraint, TranslationTargetConstraint, RotationTargetConstraint {}

    /**
     * Constraints for a {@link Waypoint}, including both translation and rotation limits.
     * 
     * @param maxVelocityMetersPerSec Maximum translational velocity in meters per second
     * @param maxAccelerationMetersPerSec2 Maximum translational acceleration in meters per second squared
     * @param maxVelocityDegPerSec Maximum rotational velocity in degrees per second
     * @param maxAccelerationDegPerSec2 Maximum rotational acceleration in degrees per second squared
     * @param minVelocityMetersPerSec Minimum translational velocity baseline in meters per second
     * @param minVelocityDegPerSec Minimum rotational velocity baseline in degrees per second
     */
    static record WaypointConstraint(
        double maxVelocityMetersPerSec,
        double maxAccelerationMetersPerSec2,
        double maxVelocityDegPerSec,
        double maxAccelerationDegPerSec2,
        double minVelocityMetersPerSec,
        double minVelocityDegPerSec
    ) implements PathElementConstraint {
        WaypointConstraint(
            double maxVelocityMetersPerSec,
            double maxAccelerationMetersPerSec2,
            double maxVelocityDegPerSec,
            double maxAccelerationDegPerSec2
        ) {
            this(
                maxVelocityMetersPerSec,
                maxAccelerationMetersPerSec2,
                maxVelocityDegPerSec,
                maxAccelerationDegPerSec2,
                0.0,
                0.0
            );
        }
    }

    /**
     * Constraints for a {@link TranslationTarget}.
     * 
     * @param maxVelocityMetersPerSec Maximum translational velocity in meters per second
     * @param maxAccelerationMetersPerSec2 Maximum translational acceleration in meters per second squared
     * @param minVelocityMetersPerSec Minimum translational velocity baseline in meters per second
     */
    static record TranslationTargetConstraint(
        double maxVelocityMetersPerSec,
        double maxAccelerationMetersPerSec2,
        double minVelocityMetersPerSec
    ) implements PathElementConstraint {
        TranslationTargetConstraint(
            double maxVelocityMetersPerSec,
            double maxAccelerationMetersPerSec2
        ) {
            this(maxVelocityMetersPerSec, maxAccelerationMetersPerSec2, 0.0);
        }
    }

    /**
     * Constraints for a {@link RotationTarget}.
     * 
     * @param maxVelocityDegPerSec Maximum rotational velocity in degrees per second
     * @param maxAccelerationDegPerSec2 Maximum rotational acceleration in degrees per second squared
     * @param minVelocityDegPerSec Minimum rotational velocity baseline in degrees per second
     */
    static record RotationTargetConstraint(
        double maxVelocityDegPerSec,
        double maxAccelerationDegPerSec2,
        double minVelocityDegPerSec
    ) implements PathElementConstraint {
        RotationTargetConstraint(
            double maxVelocityDegPerSec,
            double maxAccelerationDegPerSec2
        ) {
            this(maxVelocityDegPerSec, maxAccelerationDegPerSec2, 0.0);
        }
    }

    /**
     * A waypoint that combines both a translation target and a rotation target.
     * 
     * <p>Waypoints are the most common path element type, defining both where the robot
     * should be and what holonomic rotation it should have at that location. The rotation target
     * within a waypoint has its t_ratio automatically set based on position in the path.
     * 
     * <p>Multiple constructors are provided for convenience:
     * <pre>{@code
     * // From Pose2d
     * new Waypoint(new Pose2d(1, 2, Rotation2d.fromDegrees(90)))
     * 
     * // With custom handoff radius
     * new Waypoint(new Pose2d(1, 2, rot), 0.3)
     * 
     * // From individual components
     * new Waypoint(new Translation2d(1, 2), Rotation2d.fromDegrees(90))
     * 
     * // With profiled rotation disabled
     * new Waypoint(pose, false)
     * }</pre>
     * 
     * @param translationTarget The position target for this waypoint
     * @param rotationTarget The holonomic rotation target for this waypoint
     */
    public static record Waypoint(
        TranslationTarget translationTarget,
        RotationTarget rotationTarget
    ) implements PathElement {
        /** @return a copy with the specified intermediate handoff mode */
        public Waypoint withHandoffMode(HandoffMode mode) {
            return new Waypoint(translationTarget.withHandoffMode(mode), rotationTarget);
        }

        /** @return a copy inheriting the path/project handoff mode */
        public Waypoint withoutHandoffMode() {
            return new Waypoint(translationTarget.withoutHandoffMode(), rotationTarget);
        }

        /** @return a copy with the specified intermediate handoff distance in metres */
        public Waypoint withHandoffDistanceMeters(double distance) {
            return new Waypoint(translationTarget.withHandoffDistanceMeters(distance), rotationTarget);
        }

        /** @return a new waypoint with copied translation and rotation targets */
        public Waypoint copy() {
            return new Waypoint(translationTarget.copy(), rotationTarget.copy());
        }
        
        /**
         * Creates a waypoint from a translation, handoff radius, and rotation.
         * 
         * @param translation The target position
         * @param handoffRadius The intermediate handoff radius in meters
         * @param rotation The target holonomic rotation
         */
        public Waypoint(Translation2d translation, double handoffRadius, Rotation2d rotation) {
            this(
                new TranslationTarget(translation, Optional.of(handoffRadius)),
                new RotationTarget(rotation, 1.0, true)
            );
        }

        /**
         * Creates a waypoint with optional profiled rotation control.
         * 
         * @param translation The target position
         * @param handoffRadius The intermediate handoff radius in meters
         * @param rotation The target holonomic rotation
         * @param profiledRotation Whether to interpolate rotation along the path segment
         */
        public Waypoint(Translation2d translation, double handoffRadius, Rotation2d rotation, boolean profiledRotation) {
            this(
                new TranslationTarget(translation, Optional.of(handoffRadius)),
                new RotationTarget(rotation, 1.0, profiledRotation)
            );
        }
        
        /**
         * Creates a waypoint from a translation and rotation using default handoff radius.
         * 
         * @param translation The target position
         * @param rotation The target holonomic rotation
         */
        public Waypoint(Translation2d translation, Rotation2d rotation) {
            this(
                new TranslationTarget(translation, Optional.empty()),
                new RotationTarget(rotation, 1.0, true)
            );
        }

        /**
         * Creates a waypoint with optional profiled rotation using default handoff radius.
         * 
         * @param translation The target position
         * @param rotation The target holonomic rotation
         * @param profiledRotation Whether to interpolate rotation along the path segment
         */
        public Waypoint(Translation2d translation, Rotation2d rotation, boolean profiledRotation) {
            this(
                new TranslationTarget(translation, Optional.empty()),
                new RotationTarget(rotation, 1.0, profiledRotation)
            );
        }

        /**
         * Creates a waypoint from a Pose2d.
         * 
         * @param pose The pose containing position and rotation
         */
        public Waypoint(Pose2d pose) {
            this(pose.getTranslation(), pose.getRotation());
        }

        /**
         * Creates a waypoint from a Pose2d with custom handoff radius.
         * 
         * @param pose The pose containing position and rotation
         * @param handoffRadius The intermediate handoff radius in meters
         */
        public Waypoint(Pose2d pose, double handoffRadius) {
            this(pose.getTranslation(), handoffRadius, pose.getRotation());
        }

        /**
         * Creates a waypoint from a Pose2d with optional profiled rotation.
         * 
         * @param pose The pose containing position and rotation
         * @param profiledRotation Whether to interpolate rotation along the path segment
         */
        public Waypoint(Pose2d pose, boolean profiledRotation) {
            this(pose.getTranslation(), pose.getRotation(), profiledRotation);
        }

        /**
         * Creates a waypoint from a Pose2d with custom handoff radius and rotation profiling.
         * 
         * @param pose The pose containing position and rotation
         * @param handoffRadius The intermediate handoff radius in meters
         * @param profiledRotation Whether to interpolate rotation along the path segment
         */
        public Waypoint(Pose2d pose, double handoffRadius, boolean profiledRotation) {
            this(pose.getTranslation(), handoffRadius, pose.getRotation(), profiledRotation);
        }

        /**
         * Creates a waypoint from x, y coordinates and a rotation.
         * 
         * @param x The x coordinate in meters
         * @param y The y coordinate in meters
         * @param rot The target holonomic rotation
         */
        public Waypoint(double x, double y, Rotation2d rot) {
            this(new Translation2d(x, y), rot);
        }
    }

    /**
     * A translation target defining a position the robot should drive through.
     * 
     * <p>Translation targets form the "backbone" of a path. The robot will drive
     * through each translation target in sequence. An optional intermediate handoff
     * radius determines when the path follower switches to the next target.
     * 
     * @param translation The target position in field coordinates
     * @param intermediateHandoffRadiusMeters Optional radius at which to switch to the next target.
     *                                         If empty, the global default is used.
     */
    public static record TranslationTarget(
        Translation2d translation, 
        Optional<Double> intermediateHandoffRadiusMeters,
        Optional<HandoffMode> handoffMode
    ) implements PathElement {
        public TranslationTarget {
            java.util.Objects.requireNonNull(translation, "translation");
            java.util.Objects.requireNonNull(intermediateHandoffRadiusMeters, "handoffDistance");
            java.util.Objects.requireNonNull(handoffMode, "handoffMode");
        }

        /** Creates a target with inherited handoff mode. */
        public TranslationTarget(Translation2d translation, Optional<Double> handoffDistance) {
            this(translation, handoffDistance, Optional.empty());
        }

        /** @return a copy with the specified handoff mode */
        public TranslationTarget withHandoffMode(HandoffMode mode) {
            return new TranslationTarget(translation, intermediateHandoffRadiusMeters, Optional.of(mode));
        }

        /** @return a copy inheriting its handoff mode from path/project defaults */
        public TranslationTarget withoutHandoffMode() {
            return new TranslationTarget(translation, intermediateHandoffRadiusMeters, Optional.empty());
        }

        /** @return a copy with the specified handoff distance in metres */
        public TranslationTarget withHandoffDistanceMeters(double distance) {
            return new TranslationTarget(translation, Optional.of(distance), handoffMode);
        }
        /**
         * Creates a copy of this translation target.
         * 
         * @return A new TranslationTarget with the same values
         */
        public TranslationTarget copy() {
            return new TranslationTarget(translation, intermediateHandoffRadiusMeters, handoffMode);
        }
        
        /**
         * Creates a translation target using the default handoff radius.
         * 
         * @param translation The target position
         */
        public TranslationTarget(Translation2d translation) {
            this(translation, Optional.empty());
        }

        /**
         * Creates a translation target from x, y coordinates using default handoff radius.
         * 
         * @param x The x coordinate in meters
         * @param y The y coordinate in meters
         */
        public TranslationTarget(double x, double y) {
            this(new Translation2d(x, y));
        }

        /**
         * Creates a translation target from x, y coordinates with custom handoff radius.
         * 
         * @param x The x coordinate in meters
         * @param y The y coordinate in meters
         * @param handoffRadius The intermediate handoff radius in meters
         */
        public TranslationTarget(double x, double y, double handoffRadius) {
            this(new Translation2d(x, y), Optional.of(handoffRadius));
        }
    }

    /**
     * A rotation target defining a holonomic rotation the robot should achieve.
     * 
     * <p>Rotation targets can be placed between translation targets and use a t_ratio
     * to specify where along the segment the rotation should be achieved. A t_ratio of 0
     * means at the start of the segment, 1 means at the end.
     * 
     * <p>When {@code profiledRotation} is true, the rotation is interpolated smoothly
     * from the previous rotation to this target based on progress along the segment.
     * When false, the robot immediately targets this rotation.
     * 
     * @param rotation The target holonomic rotation
     * @param t_ratio The position along the segment (0-1) where this rotation should be achieved
     * @param profiledRotation Whether to interpolate the rotation based on position
     */
    public static record RotationTarget(
        Rotation2d rotation, 
        double t_ratio,
        boolean profiledRotation
    ) implements PathElement {
        /**
         * Creates a copy of this rotation target.
         * 
         * @return A new RotationTarget with the same values
         */
        public RotationTarget copy() {
            return new RotationTarget(rotation, t_ratio, profiledRotation);
        }
        
        /**
         * Creates a rotation target with profiled rotation enabled by default.
         * 
         * @param rotation The target holonomic rotation
         * @param t_ratio The position along the segment (0-1) where this rotation should be achieved
         */
        public RotationTarget(Rotation2d rotation, double t_ratio) {
            this(rotation, t_ratio, true);
        }
    }

    /**
     * An event trigger that fires a user-registered action at a t_ratio along a segment.
     *
     * <p>Event triggers are placed between translation targets and use their t_ratio to
     * determine when the action should fire along the segment.
     *
     * @param t_ratio The position along the segment (0-1) where this event should trigger
     * @param libKey The key used to look up the registered action
     */
    public static record EventTrigger(
        double t_ratio,
        String libKey
    ) implements PathElement {
        /**
         * Creates a copy of this event trigger.
         *
         * @return A new EventTrigger with the same values
         */
        public EventTrigger copy() {
            return new EventTrigger(t_ratio, libKey);
        }
    }

    /**
     * A constraint value that applies to a range of path elements.
     * 
     * <p>Ranged constraints allow different velocity/acceleration limits for different
     * sections of a path. The constraint applies to elements with ordinals between
     * startOrdinal and endOrdinal (inclusive).
     * 
     * @param value The constraint value (velocity or acceleration)
     * @param startOrdinal The first element ordinal this constraint applies to
     * @param endOrdinal The last element ordinal this constraint applies to
     */
    public static record RangedConstraint(
        double value,
        int startOrdinal,
        int endOrdinal
    ) {}

    /**
     * Default global constraints that apply when path-specific constraints are not set.
     * 
     * <p>These constraints are loaded from a config.json file or can be set programmatically.
     * They provide fallback values for all constraint types when individual paths don't
     * specify their own limits.
     */
    public static final class DefaultGlobalConstraints {
        private final double maxVelocityMetersPerSec;
        private final double maxAccelerationMetersPerSec2;
        private final double maxVelocityDegPerSec;
        private final double maxAccelerationDegPerSec2;
        private final double endTranslationToleranceMeters;
        private final double endRotationToleranceDeg;
        private final double intermediateHandoffRadiusMeters;

        /**
         * Creates a new set of default global constraints.
         * 
         * @param maxVelocityMetersPerSec Default maximum translational velocity (m/s)
         * @param maxAccelerationMetersPerSec2 Default maximum translational acceleration (m/s²)
         * @param maxVelocityDegPerSec Default maximum rotational velocity (deg/s)
         * @param maxAccelerationDegPerSec2 Default maximum rotational acceleration (deg/s²)
         * @param endTranslationToleranceMeters Tolerance for reaching final position (m)
         * @param endRotationToleranceDeg Tolerance for reaching final holonomic rotation (deg)
         * @param intermediateHandoffRadiusMeters Default radius for switching between targets (m)
         */
        public DefaultGlobalConstraints(
            double maxVelocityMetersPerSec,
            double maxAccelerationMetersPerSec2,
            double maxVelocityDegPerSec,
            double maxAccelerationDegPerSec2,
            double endTranslationToleranceMeters,
            double endRotationToleranceDeg,
            double intermediateHandoffRadiusMeters
        ) {
            this.maxVelocityMetersPerSec = maxVelocityMetersPerSec;
            this.maxAccelerationMetersPerSec2 = maxAccelerationMetersPerSec2;
            this.maxVelocityDegPerSec = maxVelocityDegPerSec;
            this.maxAccelerationDegPerSec2 = maxAccelerationDegPerSec2;
            this.endTranslationToleranceMeters = endTranslationToleranceMeters;
            this.endRotationToleranceDeg = endRotationToleranceDeg;
            this.intermediateHandoffRadiusMeters = intermediateHandoffRadiusMeters;
        }

        /**
         * Creates a deep copy of these constraints.
         * 
         * @return A new DefaultGlobalConstraints with the same values
         */
        public DefaultGlobalConstraints copy() {
            return new DefaultGlobalConstraints(
                maxVelocityMetersPerSec,
                maxAccelerationMetersPerSec2,
                maxVelocityDegPerSec,
                maxAccelerationDegPerSec2,
                endTranslationToleranceMeters,
                endRotationToleranceDeg,
                intermediateHandoffRadiusMeters
            );
        }

        /** @return The default maximum translational velocity in meters per second */
        public double getMaxVelocityMetersPerSec() { return maxVelocityMetersPerSec; }

        /** @return The default maximum translational acceleration in meters per second squared */
        public double getMaxAccelerationMetersPerSec2() { return maxAccelerationMetersPerSec2; }

        /** @return The default maximum rotational velocity in degrees per second */
        public double getMaxVelocityDegPerSec() { return maxVelocityDegPerSec; }

        /** @return The default maximum rotational acceleration in degrees per second squared */
        public double getMaxAccelerationDegPerSec2() { return maxAccelerationDegPerSec2; }

        /** @return The tolerance for reaching the final translation in meters */
        public double getEndTranslationToleranceMeters() { return endTranslationToleranceMeters; }

        /** @return The tolerance for reaching the final rotation in degrees */
        public double getEndRotationToleranceDeg() { return endRotationToleranceDeg; }

        /** @return The default intermediate handoff radius in meters */
        public double getIntermediateHandoffRadiusMeters() { return intermediateHandoffRadiusMeters; }
    }
    
    /**
     * Path-specific constraints that override global defaults.
     * 
     * <p>This class allows setting velocity and acceleration limits that apply only
     * to a specific path. Constraints can be set as single values (applying to the
     * entire path) or as ranged constraints (applying to specific sections).
     * 
     * <p>Uses a fluent API for easy configuration:
     * <pre>{@code
     * PathConstraints constraints = new PathConstraints()
     *     .setMaxVelocityMetersPerSec(3.0)
     *     .setMaxAccelerationMetersPerSec2(2.0)
     *     .setEndTranslationToleranceMeters(0.05);
     * }</pre>
     */
    public static final class PathConstraints {
        private Optional<ArrayList<RangedConstraint>> maxVelocityMetersPerSec = Optional.empty();
        private Optional<ArrayList<RangedConstraint>> maxAccelerationMetersPerSec2 = Optional.empty();
        private Optional<ArrayList<RangedConstraint>> maxVelocityDegPerSec = Optional.empty();
        private Optional<ArrayList<RangedConstraint>> maxAccelerationDegPerSec2 = Optional.empty();
        private Optional<ArrayList<RangedConstraint>> minVelocityMetersPerSec = Optional.empty();
        private Optional<ArrayList<RangedConstraint>> minVelocityDegPerSec = Optional.empty();
        private Optional<Double> endTranslationToleranceMeters = Optional.empty();
        private Optional<Double> endRotationToleranceDeg = Optional.empty();

        /**
         * Creates an empty PathConstraints with no overrides set.
         */
        public PathConstraints() {}

        /**
         * Sets a global maximum translational velocity for the entire path.
         * 
         * @param value The maximum velocity in meters per second
         * @return This PathConstraints for chaining
         */
        public PathConstraints setMaxVelocityMetersPerSec(double value) {
            ArrayList<RangedConstraint> list = new ArrayList<>();
            list.add(new RangedConstraint(value, 0, Integer.MAX_VALUE));
            this.maxVelocityMetersPerSec = Optional.of(list);
            return this;
        }
        
        /**
         * Sets ranged maximum translational velocity constraints.
         * 
         * @param constraints The ranged constraints to apply
         * @return This PathConstraints for chaining
         */
        public PathConstraints setMaxVelocityMetersPerSec(RangedConstraint... constraints) {
            this.maxVelocityMetersPerSec = Optional.of(new ArrayList<>(List.of(constraints)));
            return this;
        }

        /**
         * Sets a global maximum translational acceleration for the entire path.
         * 
         * @param value The maximum acceleration in meters per second squared
         * @return This PathConstraints for chaining
         */
        public PathConstraints setMaxAccelerationMetersPerSec2(double value) {
            ArrayList<RangedConstraint> list = new ArrayList<>();
            list.add(new RangedConstraint(value, 0, Integer.MAX_VALUE));
            this.maxAccelerationMetersPerSec2 = Optional.of(list);
            return this;
        }

        /**
         * Sets ranged maximum translational acceleration constraints.
         * 
         * @param constraints The ranged constraints to apply
         * @return This PathConstraints for chaining
         */
        public PathConstraints setMaxAccelerationMetersPerSec2(RangedConstraint... constraints) {
            this.maxAccelerationMetersPerSec2 = Optional.of(new ArrayList<>(List.of(constraints)));
            return this;
        }

        /**
         * Sets a global maximum rotational velocity for the entire path.
         * 
         * @param value The maximum angular velocity in degrees per second
         * @return This PathConstraints for chaining
         */
        public PathConstraints setMaxVelocityDegPerSec(double value) {
            ArrayList<RangedConstraint> list = new ArrayList<>();
            list.add(new RangedConstraint(value, 0, Integer.MAX_VALUE));
            this.maxVelocityDegPerSec = Optional.of(list);
            return this;
        }

        /**
         * Sets ranged maximum rotational velocity constraints.
         * 
         * @param constraints The ranged constraints to apply
         * @return This PathConstraints for chaining
         */
        public PathConstraints setMaxVelocityDegPerSec(RangedConstraint... constraints) {
            this.maxVelocityDegPerSec = Optional.of(new ArrayList<>(List.of(constraints)));
            return this;
        }

        /**
         * Sets a global maximum rotational acceleration for the entire path.
         * 
         * @param value The maximum angular acceleration in degrees per second squared
         * @return This PathConstraints for chaining
         */
        public PathConstraints setMaxAccelerationDegPerSec2(double value) {
            ArrayList<RangedConstraint> list = new ArrayList<>();
            list.add(new RangedConstraint(value, 0, Integer.MAX_VALUE));
            this.maxAccelerationDegPerSec2 = Optional.of(list);
            return this;
        }

        /**
         * Sets ranged maximum rotational acceleration constraints.
         * 
         * @param constraints The ranged constraints to apply
         * @return This PathConstraints for chaining
         */
        public PathConstraints setMaxAccelerationDegPerSec2(RangedConstraint... constraints) {
            this.maxAccelerationDegPerSec2 = Optional.of(new ArrayList<>(List.of(constraints)));
            return this;
        }

        /**
         * Sets a global minimum translational velocity baseline for the entire path.
         *
         * <p>This is an advanced feature. Minimum output baselines can help overcome
         * static friction, but values that are too high may cause overshoot or
         * controller oscillation near targets.
         *
         * @param value The minimum velocity in meters per second
         * @return This PathConstraints for chaining
         */
        public PathConstraints setMinVelocityMetersPerSec(double value) {
            ArrayList<RangedConstraint> list = new ArrayList<>();
            list.add(new RangedConstraint(value, 0, Integer.MAX_VALUE));
            this.minVelocityMetersPerSec = Optional.of(list);
            return this;
        }

        /**
         * Sets ranged minimum translational velocity baseline constraints.
         *
         * <p>This is an advanced feature. Minimum output baselines can help overcome
         * static friction, but values that are too high may cause overshoot or
         * controller oscillation near targets.
         *
         * @param constraints The ranged constraints to apply
         * @return This PathConstraints for chaining
         */
        public PathConstraints setMinVelocityMetersPerSec(RangedConstraint... constraints) {
            this.minVelocityMetersPerSec = Optional.of(new ArrayList<>(List.of(constraints)));
            return this;
        }

        /**
         * Sets a global minimum rotational velocity baseline for the entire path.
         *
         * <p>This is an advanced feature. Minimum output baselines can help overcome
         * static friction, but values that are too high may cause overshoot or
         * controller oscillation near targets.
         *
         * @param value The minimum angular velocity in degrees per second
         * @return This PathConstraints for chaining
         */
        public PathConstraints setMinVelocityDegPerSec(double value) {
            ArrayList<RangedConstraint> list = new ArrayList<>();
            list.add(new RangedConstraint(value, 0, Integer.MAX_VALUE));
            this.minVelocityDegPerSec = Optional.of(list);
            return this;
        }

        /**
         * Sets ranged minimum rotational velocity baseline constraints.
         *
         * <p>This is an advanced feature. Minimum output baselines can help overcome
         * static friction, but values that are too high may cause overshoot or
         * controller oscillation near targets.
         *
         * @param constraints The ranged constraints to apply
         * @return This PathConstraints for chaining
         */
        public PathConstraints setMinVelocityDegPerSec(RangedConstraint... constraints) {
            this.minVelocityDegPerSec = Optional.of(new ArrayList<>(List.of(constraints)));
            return this;
        }

        /**
         * Sets the translation tolerance for path completion.
         * 
         * @param value The tolerance in meters
         * @return This PathConstraints for chaining
         */
        public PathConstraints setEndTranslationToleranceMeters(double value) {
            this.endTranslationToleranceMeters = Optional.of(value);
            return this;
        }

        /**
         * Sets the rotation tolerance for path completion.
         * 
         * @param value The tolerance in degrees
         * @return This PathConstraints for chaining
         */
        public PathConstraints setEndRotationToleranceDeg(double value) {
            this.endRotationToleranceDeg = Optional.of(value);
            return this;
        }

        /** @return Optional ranged constraints for maximum translational velocity */
        public Optional<ArrayList<RangedConstraint>> getMaxVelocityMetersPerSec() { return maxVelocityMetersPerSec.map(list -> new ArrayList<>(list)); }
        
        /** @return Optional ranged constraints for maximum translational acceleration */
        public Optional<ArrayList<RangedConstraint>> getMaxAccelerationMetersPerSec2() { return maxAccelerationMetersPerSec2.map(list -> new ArrayList<>(list)); }
        
        /** @return Optional ranged constraints for maximum rotational velocity */
        public Optional<ArrayList<RangedConstraint>> getMaxVelocityDegPerSec() { return maxVelocityDegPerSec.map(list -> new ArrayList<>(list)); }
        
        /** @return Optional ranged constraints for maximum rotational acceleration */
        public Optional<ArrayList<RangedConstraint>> getMaxAccelerationDegPerSec2() { return maxAccelerationDegPerSec2.map(list -> new ArrayList<>(list)); }

        /** @return Optional ranged constraints for minimum translational velocity */
        public Optional<ArrayList<RangedConstraint>> getMinVelocityMetersPerSec() { return minVelocityMetersPerSec.map(list -> new ArrayList<>(list)); }

        /** @return Optional ranged constraints for minimum rotational velocity */
        public Optional<ArrayList<RangedConstraint>> getMinVelocityDegPerSec() { return minVelocityDegPerSec.map(list -> new ArrayList<>(list)); }
        
        /** @return Optional end translation tolerance override */
        public Optional<Double> getEndTranslationToleranceMeters() { return endTranslationToleranceMeters; }
        
        /** @return Optional end rotation tolerance override */
        public Optional<Double> getEndRotationToleranceDeg() { return endRotationToleranceDeg; }

        /**
         * Copies these constraints, restricting their velocity and acceleration ranges to the
         * inclusive interval {@code startOrdinal..endOrdinal}. This is useful for applying a
         * reusable preset to different sections of different paths:
         *
         * <pre>{@code
         * var intake = new PathConstraints()
         *     .setMaxVelocityMetersPerSec(1.5)
         *     .setMaxAccelerationMetersPerSec2(2.0);
         * var transit = new PathConstraints().setMaxVelocityMetersPerSec(4.0);
         * path.setPathConstraints(PathConstraints.combine(
         *     intake.inRange(1, 3),
         *     transit.inRange(4, 6)
         * ));
         * // Translation ordinals 1-3 use intake's speed and acceleration.
         * // Ordinals 4-6 use transit's speed and the default acceleration.
         * // Ordinal 0 uses the project defaults. Neither preset was changed.
         * }</pre>
         *
         * <p>Ordinals start at zero. Translational limits count translation targets and waypoints;
         * rotational limits count rotation targets and waypoints. Events count toward neither.
         * The same interval is applied independently to those two ordinal sequences, not to raw
         * element indices. A measured/ghost start is not an authored element and adds no ordinal.
         *
         * <p>Existing ranges are intersected, not shifted: a range of 1-4 restricted to 3-6 becomes
         * 3-4. Disjoint ranges are removed; unset properties remain unset. End tolerances are copied
         * unchanged and remain path-wide, even if no velocity/acceleration range intersects.
         *
         * @param startOrdinal first included ordinal (nonnegative)
         * @param endOrdinal last included ordinal, at least startOrdinal
         * @return an independent constraint set; this object is unchanged
         * @throws IllegalArgumentException if the requested interval is negative or reversed
         * @see #combine(PathConstraints...)
         */
        public PathConstraints inRange(int startOrdinal, int endOrdinal) {
            if (startOrdinal < 0 || endOrdinal < startOrdinal) {
                throw new IllegalArgumentException("Constraint range requires 0 <= startOrdinal <= endOrdinal");
            }
            PathConstraints result = copy();
            result.maxVelocityMetersPerSec = intersect(maxVelocityMetersPerSec, startOrdinal, endOrdinal);
            result.maxAccelerationMetersPerSec2 = intersect(maxAccelerationMetersPerSec2, startOrdinal, endOrdinal);
            result.maxVelocityDegPerSec = intersect(maxVelocityDegPerSec, startOrdinal, endOrdinal);
            result.maxAccelerationDegPerSec2 = intersect(maxAccelerationDegPerSec2, startOrdinal, endOrdinal);
            result.minVelocityMetersPerSec = intersect(minVelocityMetersPerSec, startOrdinal, endOrdinal);
            result.minVelocityDegPerSec = intersect(minVelocityDegPerSec, startOrdinal, endOrdinal);
            return result;
        }

        /**
         * Combines constraint sets into a new object, preserving argument and range order.
         * Inputs are not modified or retained as mutable lists.
         *
         * <p>For each constraint type, the first matching range wins. Values are not added,
         * averaged, or automatically reduced to the strictest limit. Put section overrides before
         * a whole-path fallback:
         *
         * <pre>{@code
         * var slow = new PathConstraints().setMaxVelocityMetersPerSec(1.5);
         * var cruise = new PathConstraints().setMaxVelocityMetersPerSec(4.0);
         * var limits = PathConstraints.combine(slow.inRange(1, 3), cruise);
         * // Speed is 1.5 m/s at ordinals 1-3 and 4.0 m/s elsewhere.
         * // Reversing the arguments would make cruise win everywhere.
         * }</pre>
         *
         * <p>Maximum/minimum translation and angular speeds, and both accelerations, are combined
         * independently. Each end tolerance uses the first explicitly supplied value and remains
         * path-wide. With no matching range or explicit tolerance, normal project defaults apply.
         * Subsequent edits to an input do not update an already combined result; combine again
         * and call {@link Path#setPathConstraints(PathConstraints)} to apply changed presets.
         *
         * @param constraints constraint sets in precedence order (no null entries)
         * @return independent combined constraints, or empty constraints for no arguments
         * @throws NullPointerException if the array or an entry is null
         */
        public static PathConstraints combine(PathConstraints... constraints) {
            java.util.Objects.requireNonNull(constraints, "constraints");
            PathConstraints result = new PathConstraints();
            for (PathConstraints source : constraints) {
                java.util.Objects.requireNonNull(source, "constraint set");
                result.maxVelocityMetersPerSec = append(result.maxVelocityMetersPerSec, source.maxVelocityMetersPerSec);
                result.maxAccelerationMetersPerSec2 = append(result.maxAccelerationMetersPerSec2, source.maxAccelerationMetersPerSec2);
                result.maxVelocityDegPerSec = append(result.maxVelocityDegPerSec, source.maxVelocityDegPerSec);
                result.maxAccelerationDegPerSec2 = append(result.maxAccelerationDegPerSec2, source.maxAccelerationDegPerSec2);
                result.minVelocityMetersPerSec = append(result.minVelocityMetersPerSec, source.minVelocityMetersPerSec);
                result.minVelocityDegPerSec = append(result.minVelocityDegPerSec, source.minVelocityDegPerSec);
                if (result.endTranslationToleranceMeters.isEmpty()) result.endTranslationToleranceMeters = source.endTranslationToleranceMeters;
                if (result.endRotationToleranceDeg.isEmpty()) result.endRotationToleranceDeg = source.endRotationToleranceDeg;
            }
            return result;
        }

        private static Optional<ArrayList<RangedConstraint>> intersect(
            Optional<ArrayList<RangedConstraint>> ranges, int start, int end
        ) {
            return ranges.map(values -> {
                ArrayList<RangedConstraint> result = new ArrayList<>();
                for (RangedConstraint value : values) {
                    int low = Math.max(start, value.startOrdinal());
                    int high = Math.min(end, value.endOrdinal());
                    if (low <= high) result.add(new RangedConstraint(value.value(), low, high));
                }
                return result;
            });
        }

        private static Optional<ArrayList<RangedConstraint>> append(
            Optional<ArrayList<RangedConstraint>> first, Optional<ArrayList<RangedConstraint>> second
        ) {
            if (first.isEmpty() && second.isEmpty()) return Optional.empty();
            ArrayList<RangedConstraint> result = new ArrayList<>();
            first.ifPresent(result::addAll);
            second.ifPresent(result::addAll);
            return Optional.of(result);
        }

        /**
         * Creates a deep copy of these constraints.
         * 
         * @return A new PathConstraints with the same values
         */
        public PathConstraints copy() {
            PathConstraints c = new PathConstraints();
            if (maxVelocityMetersPerSec.isPresent()) c.setMaxVelocityMetersPerSec(maxVelocityMetersPerSec.get().toArray(new RangedConstraint[0]));
            if (maxAccelerationMetersPerSec2.isPresent()) c.setMaxAccelerationMetersPerSec2(maxAccelerationMetersPerSec2.get().toArray(new RangedConstraint[0]));
            if (maxVelocityDegPerSec.isPresent()) c.setMaxVelocityDegPerSec(maxVelocityDegPerSec.get().toArray(new RangedConstraint[0]));
            if (maxAccelerationDegPerSec2.isPresent()) c.setMaxAccelerationDegPerSec2(maxAccelerationDegPerSec2.get().toArray(new RangedConstraint[0]));
            if (minVelocityMetersPerSec.isPresent()) c.setMinVelocityMetersPerSec(minVelocityMetersPerSec.get().toArray(new RangedConstraint[0]));
            if (minVelocityDegPerSec.isPresent()) c.setMinVelocityDegPerSec(minVelocityDegPerSec.get().toArray(new RangedConstraint[0]));
            if (endTranslationToleranceMeters.isPresent()) c.setEndTranslationToleranceMeters(endTranslationToleranceMeters.get());
            if (endRotationToleranceDeg.isPresent()) c.setEndRotationToleranceDeg(endRotationToleranceDeg.get());
            return c;
        }
    }

    private List<PathElement> pathElements;
    private PathConstraints pathConstraints;
    // The numerical defaults are loaded lazily when the first path needs them.
    private record DefaultState(DefaultGlobalConstraints constraints, HandoffMode handoffMode) {}
    private static volatile DefaultState defaultState = new DefaultState(null, HandoffMode.RADIUS);
    private boolean flipped = false;
    private boolean mirrored = false;
    private Optional<HandoffMode> handoffMode = Optional.empty();
    private DriveDirection tankDriveDirection = DriveDirection.FORWARD;

    /** Sets the project-wide fallback; active followers retain their resolved settings. */
    public static void setDefaultHandoffMode(HandoffMode mode) {
        defaultState = new DefaultState(defaultState.constraints(), java.util.Objects.requireNonNull(mode, "mode"));
    }

    /**
     * Selects the default travel direction for tank followers of this path.
     * Backward means rear-first travel in the same element order; headings and pose reset are unchanged.
     * Active executions retain their snapshot. Holonomic followers ignore this property.
     * @param direction forward or backward travel (not null)
     * @return this path
     */
    public Path setTankDriveDirection(DriveDirection direction) {
        tankDriveDirection = java.util.Objects.requireNonNull(direction, "direction");
        return this;
    }

    /** @return this path's tank travel direction; forward for paths without a saved direction */
    public DriveDirection getTankDriveDirection() { return tankDriveDirection; }

    /** @return the project-wide default handoff mode */
    public static HandoffMode getDefaultHandoffMode() { return defaultState.handoffMode(); }

    /** @return this path, overriding the project default */
    public Path setHandoffMode(HandoffMode mode) { handoffMode = Optional.of(mode); return this; }

    /** @return this path, inheriting the project default */
    public Path clearHandoffMode() { handoffMode = Optional.empty(); return this; }

    /** @return this path's optional handoff-mode override */
    public Optional<HandoffMode> getHandoffMode() { return handoffMode; }
    
    /**
     * Creates a new Path with the specified elements, constraints, and global defaults.
     * 
     * @param pathElements The list of path elements defining the path
     * @param constraints Path-specific constraints (can be null for defaults)
     * @param defaultGlobalConstraints Global constraint defaults (can be null to load from config)
     * @throws IllegalArgumentException if pathElements is null
     * @throws RuntimeException if defaultGlobalConstraints is null and config cannot be loaded
     */
    public Path(List<PathElement> pathElements, PathConstraints constraints, DefaultGlobalConstraints defaultGlobalConstraints) {
        this(pathElements, constraints, defaultGlobalConstraints, null);
    }

    Path(List<PathElement> pathElements, PathConstraints constraints, DefaultGlobalConstraints globals, HandoffMode mode) {
        if (pathElements == null) throw new IllegalArgumentException("pathElements cannot be null");
        ProjectDefaults settings = globals == null ? loadProjectDefaults(JsonPathCodec.projectRoot())
            : new ProjectDefaults(globals, mode == null ? getDefaultHandoffMode() : mode);
        this.pathElements = new ArrayList<>(pathElements);
        this.pathConstraints = constraints == null ? new PathConstraints() : constraints.copy();
        setProjectDefaults(settings);
    }

    /**
     * Creates a new Path from varargs path elements using current global defaults.
     * 
     * @param pathElements The path elements defining the path
     */
    public Path(PathElement... pathElements) {
        this(List.of(pathElements), null, defaultState.constraints());
    }

    /**
     * Creates a new Path with custom constraints from varargs path elements.
     * 
     * @param constraints The path-specific constraints
     * @param pathElements The path elements defining the path
     */
    public Path(PathConstraints constraints, PathElement... pathElements) {
        this(List.of(pathElements), constraints, defaultState.constraints());
    }

    /**
     * Creates a new Path from an array of path elements with constraints and global defaults.
     * 
     * @param pathElements The array of path elements
     * @param constraints The path-specific constraints
     * @param defaultGlobalConstraints The global constraint defaults
     */
    public Path(PathElement[] pathElements, PathConstraints constraints, DefaultGlobalConstraints defaultGlobalConstraints) {
        this(List.of(pathElements), constraints, defaultGlobalConstraints);
    }

    /**
     * Creates a new Path from a list of path elements using current global defaults.
     * 
     * @param pathElements The list of path elements
     */
    public Path(List<PathElement> pathElements) {
        this(pathElements, null, defaultState.constraints());
    }

    /**
     * Creates a new Path from a list with custom constraints.
     * 
     * @param pathElements The list of path elements
     * @param constraints The path-specific constraints
     */
    public Path(List<PathElement> pathElements, PathConstraints constraints) {
        this(pathElements, constraints, defaultState.constraints());
    }

    /**
     * Loads a Path from a JSON file in the specified autos directory.
     * 
     * @param autosDir The directory containing the autos folder structure
     * @param pathFileName The name of the path file (without .json extension)
     */
    public Path(File autosDir, String pathFileName) {
        Path loaded = JsonPathCodec.loadPath(autosDir, pathFileName.endsWith(".json") ? pathFileName : pathFileName + ".json");
        this.pathElements = loaded.pathElements;
        this.pathConstraints = loaded.pathConstraints;
        this.handoffMode = loaded.handoffMode;
        this.tankDriveDirection = loaded.tankDriveDirection;
        // globals are static and already copied

    }

    /**
     * Loads a Path from a JSON file in the default project root directory.
     * 
     * <p>The file is loaded from {@code deploy/autos/paths/<pathFileName>.json}.
     * 
     * @param pathFileName The name of the path file (without .json extension)
     */
    public Path(String pathFileName) {
        this(JsonPathCodec.projectRoot(), pathFileName);
    }

    /**
     * Loads authored path data from JSON text; executable structure is validated when following.
     * @param json path JSON text
     * @param defaults project defaults, or null to load project configuration
     * @return the authored path
     */
    public static Path fromJson(String json, DefaultGlobalConstraints defaults) {
        return JsonPathCodec.loadPathFromJsonString(json, defaults);
    }

    /**
     * Reads numerical defaults from config.json without applying them or changing handoff mode.
     * Use {@link #loadProjectDefaults(File)} to obtain the complete project configuration.
     * @param autosDir project directory containing config.json
     * @return loaded motion constraints
     */
    public static DefaultGlobalConstraints loadGlobalConstraints(File autosDir) {
        return JsonPathCodec.loadGlobalConstraints(autosDir);
    }

    /**
     * Reads numerical constraints and the default handoff mode from config.json without applying them.
     *
     * <pre>{@code
     * var defaults = Path.loadProjectDefaults(autosDirectory);
     * Path.setProjectDefaults(defaults);
     * }</pre>
     *
     * @param autosDir directory containing config.json
     * @return the complete project defaults; a missing handoff mode resolves to Radius
     * @throws RuntimeException if the configuration cannot be read or parsed
     */
    public static ProjectDefaults loadProjectDefaults(File autosDir) {
        return JsonPathCodec.loadProjectDefaults(autosDir);
    }

    /**
     * Installs numerical defaults and the handoff mode together for subsequent path executions.
     * Does not modify authored overrides or active executions. Numerical validity is checked
     * when following or explicitly calling {@link #isValid()}, not while configuring defaults.
     *
     * @param defaults complete project defaults (not null)
     */
    public static void setProjectDefaults(ProjectDefaults defaults) {
        java.util.Objects.requireNonNull(defaults, "defaults");
        defaultState = new DefaultState(defaults.constraints(), defaults.handoffMode());
    }

    static ProjectDefaults currentProjectDefaults() {
        DefaultState state = defaultState;
        return new ProjectDefaults(state.constraints(), state.handoffMode());
    }

    /**
     * Checks the current elements and resolved motion constraints without changing the path or logging.
     * Following uses the same validation on a fresh snapshot at each execution. Construction and
     * editing do not require a valid path. Follower-specific options are checked at execution instead.
     *
     * @return whether the current path can be prepared for following
     */
    public boolean isValid() {
        return PreparedPath.validationError(this).isEmpty();
    }

    Optional<String> elementValidationError() {
        if (pathElements.isEmpty()) return Optional.of("Path has no destination");
        PathElement last = pathElements.getLast();
        if (!(last instanceof Waypoint || last instanceof TranslationTarget)) {
            return Optional.of("The last element must be a waypoint or translation target");
        }
        for (int i = 0; i < pathElements.size(); i++) {
            PathElement element = pathElements.get(i);
            if (element == null) return Optional.of("Element " + i + " is null");
            if (element instanceof Waypoint waypoint && (waypoint.translationTarget() == null || waypoint.rotationTarget() == null)) {
                return Optional.of("Element " + i + " has an incomplete waypoint");
            }
            TranslationTarget translation = element instanceof Waypoint w ? w.translationTarget()
                : element instanceof TranslationTarget t ? t : null;
            RotationTarget rotation = element instanceof Waypoint w ? w.rotationTarget()
                : element instanceof RotationTarget r ? r : null;
            if (translation != null && (translation.translation() == null
                || !Double.isFinite(translation.translation().getX())
                || !Double.isFinite(translation.translation().getY())
                || translation.intermediateHandoffRadiusMeters() == null || translation.handoffMode() == null
                || translation.intermediateHandoffRadiusMeters().filter(v -> !Double.isFinite(v) || v < 0).isPresent())) {
                return Optional.of("Element " + i + " has an invalid translation or handoff distance");
            }
            if (rotation != null && (rotation.rotation() == null
                || !Double.isFinite(rotation.rotation().getRadians())
                || !Double.isFinite(rotation.t_ratio()) || rotation.t_ratio() < 0 || rotation.t_ratio() > 1)) {
                return Optional.of("Element " + i + " has an invalid rotation or progress");
            }
            if (element instanceof EventTrigger event && (!Double.isFinite(event.t_ratio())
                || event.t_ratio() < 0 || event.t_ratio() > 1 || event.libKey() == null)) {
                return Optional.of("Element " + i + " has an invalid event key or progress");
            }
        }
        return Optional.empty();
    }

    // A single destination supplies no authored start. Neither do leading rotation/event elements.
    boolean hasAuthoredStart() {
        return pathElements.size() > 1
            && (pathElements.getFirst() instanceof Waypoint || pathElements.getFirst() instanceof TranslationTarget);
    }

    /**
     * Gets a copy of the current default global constraints.
     * 
     * @return A copy of the default global constraints
     */
    public DefaultGlobalConstraints getDefaultGlobalConstraints() {
        return defaultState.constraints().copy();
    }

    /**
     * Sets the default global constraints used by all paths.
     * 
     * <p>This is a static method that affects subsequent executions of all paths. Active executions keep their resolved defaults.
     * 
     * @param defaultGlobalConstraints The new global constraints
     * @throws IllegalArgumentException if defaultGlobalConstraints is null
     */
    public static void setDefaultGlobalConstraints(DefaultGlobalConstraints defaultGlobalConstraints) {
        if (defaultGlobalConstraints == null) {
            throw new IllegalArgumentException("defaultGlobalConstraints cannot be null");
        }
        defaultState = new DefaultState(defaultGlobalConstraints.copy(), getDefaultHandoffMode());
    }

    /**
     * Gets a copy of this path's constraints.
     * 
     * @return A copy of the path constraints
     */
    public PathConstraints getPathConstraints() { return pathConstraints.copy(); }

    /**
     * Sets this path's constraints.
     * 
     * @param pathConstraints The new path constraints
     * @throws IllegalArgumentException if pathConstraints is null
     */
    public void setPathConstraints(PathConstraints pathConstraints) { 
        if (pathConstraints == null) {
            throw new IllegalArgumentException("pathConstraints cannot be null");
        }
        this.pathConstraints = pathConstraints.copy(); 
    }

    /**
     * Gets the end translation tolerance, using path-specific if set, otherwise global default.
     * 
     * @return The end translation tolerance in meters
     */
    public double getEndTranslationToleranceMeters() {
        return pathConstraints.getEndTranslationToleranceMeters().orElse(defaultState.constraints().getEndTranslationToleranceMeters());
    }

    /**
     * Gets the end rotation tolerance, using path-specific if set, otherwise global default.
     * 
     * @return The end rotation tolerance in degrees
     */
    public double getEndRotationToleranceDeg() {
        return pathConstraints.getEndRotationToleranceDeg().orElse(defaultState.constraints().getEndRotationToleranceDeg());
    }

    /**
     * Adds a path element to the end of this path.
     * 
     * @param pathElement The element to add
     * @return This path for chaining
     */
    public Path addPathElement(PathElement pathElement) {
        pathElements.add(pathElement);
        return this;
    }

    /**
     * Gets a path element by index.
     * 
     * @param index The index of the element
     * @return The path element at the specified index
     * @throws IndexOutOfBoundsException if index is out of range
     */
    public PathElement getElement(int index) {
        if (index >= 0 && index < pathElements.size()) {
            return pathElements.get(index);
        }
        throw new IndexOutOfBoundsException("Index out of range");
    }

    /**
     * Sets a path element at the specified index.
     * 
     * @param index The index to set
     * @param element The new element
     * @throws IndexOutOfBoundsException if index is out of range
     */
    public void setElement(int index, PathElement element) {
        if (index >= 0 && index < pathElements.size()) {
            pathElements.set(index, element);
            return;
        }
        throw new IndexOutOfBoundsException("Index out of range");
    }

    /**
     * Removes and returns a path element at the specified index.
     * 
     * @param index The index of the element to remove
     * @return The removed element
     * @throws IndexOutOfBoundsException if index is out of range
     */
    public PathElement removeElement(int index) {
        if (index >= 0 && index < pathElements.size()) {
            return pathElements.remove(index);
        }
        throw new IndexOutOfBoundsException("Index out of range");
    }

    /**
     * Reorders the path elements according to the specified order.
     * 
     * @param newOrder List of indices specifying the new order
     * @return This path for chaining
     * @throws IllegalArgumentException if newOrder is not a permutation of all existing indices
     */
    public Path reorderElements(List<Integer> newOrder) {
        if (newOrder == null || newOrder.size() != pathElements.size()) {
            throw new IllegalArgumentException("New order must match elements length");
        }
        boolean[] seen = new boolean[pathElements.size()];
        List<PathElement> reordered = new ArrayList<>(pathElements.size());
        for (Integer i : newOrder) {
            if (i == null || i < 0 || i >= seen.length) {
                throw new IllegalArgumentException("New order contains an invalid element index: " + i);
            }
            if (seen[i]) {
                throw new IllegalArgumentException("New order repeats element index " + i);
            }
            seen[i] = true;
            reordered.add(pathElements.get(i));
        }
        this.pathElements = reordered;
        return this;
    }

    /**
     * Gets a copy of all path elements.
     * 
     * @return A new list containing all path elements
     */
    public List<PathElement> getPathElements() { return new ArrayList<>(pathElements); }

    /**
     * Gets the ordered translations that make up this path's polyline.
     *
     * <p>The translations are taken from translation targets, with waypoints
     * expanded automatically. Rotation targets and event triggers are ignored
     * because they do not add vertices to the path polyline. Coordinates reflect
     * this path's current flip/mirror state.
     *
     * @return Ordered translations for this path, or an empty list if the path is invalid
     */
    public List<Translation2d> getTranslations() {
        List<Translation2d> translations = new ArrayList<>();
        if (elementValidationError().isPresent()) {
            return translations;
        }

        for (PathElement element : pathElements) {
            if (element instanceof TranslationTarget translationTarget) {
                translations.add(translationTarget.translation());
            } else if (element instanceof Waypoint waypoint) {
                translations.add(waypoint.translationTarget().translation());
            }
        }
        return translations;
    }

    /**
     * Sets the path elements, replacing all existing elements.
     * 
     * @param pathElements The new list of path elements
     * @throws IllegalArgumentException if pathElements is null
     */
    public void setPathElements(List<PathElement> pathElements) { 
        if (pathElements == null) {
            throw new IllegalArgumentException("pathElements cannot be null");
        }
        this.pathElements = new ArrayList<>(pathElements); 
    }

    private record ResolvedConstraintValue(double maxValue, double minValue) {}

    private Optional<Double> findRangedConstraintValue(
        Optional<ArrayList<RangedConstraint>> constraints,
        int ordinal
    ) {
        if (constraints.isEmpty()) {
            return Optional.empty();
        }
        for (RangedConstraint constraint : constraints.get()) {
            if (constraint.startOrdinal() <= ordinal && constraint.endOrdinal() >= ordinal) {
                return Optional.of(constraint.value());
            }
        }
        return Optional.empty();
    }

    private ResolvedConstraintValue resolveConstraintValue(
        String label,
        int ordinal,
        Optional<ArrayList<RangedConstraint>> maxConstraints,
        Optional<ArrayList<RangedConstraint>> minConstraints,
        double globalMaxValue,
        boolean reportWarnings
    ) {
        double maxValue = findRangedConstraintValue(maxConstraints, ordinal).orElse(globalMaxValue);
        double minValue = findRangedConstraintValue(minConstraints, ordinal).orElse(0.0);

        if (minValue > maxValue) {
            if (reportWarnings) logger.log(
                Level.WARNING,
                "Path constraint conflict for {0} at ordinal {1}: minimum {2} exceeds maximum {3}; using global default {4} and disabling the minimum baseline",
                new Object[] { label, ordinal, minValue, maxValue, globalMaxValue }
            );
            return new ResolvedConstraintValue(globalMaxValue, 0.0);
        }

        return new ResolvedConstraintValue(maxValue, minValue);
    }

    private double resolveMaxConstraintValue(
        Optional<ArrayList<RangedConstraint>> maxConstraints,
        int ordinal,
        double globalMaxValue
    ) {
        return findRangedConstraintValue(maxConstraints, ordinal).orElse(globalMaxValue);
    }

    /**
     * Gets all path elements paired with their computed constraints.
     * 
     * <p>This method resolves which constraints apply to each element based on
     * path-specific constraints and global defaults.
     * 
     * @return List of (PathElement, PathElementConstraint) pairs
     */
    List<Pair<PathElement, PathElementConstraint>> getPathElementsWithConstraints() {
        return getPathElementsWithConstraints(defaultState.constraints(), true);
    }

    private List<Pair<PathElement, PathElementConstraint>> getPathElementsWithConstraints(
        DefaultGlobalConstraints defaultGlobalConstraints, boolean reportWarnings
    ) {
        if (elementValidationError().isPresent()) {
            return new ArrayList<>();
        }
        
        List<Pair<PathElement, PathElementConstraint>> elementsWithConstraints = new ArrayList<>();
        int translationOrdinal = 0;
        int rotationOrdinal = 0;
        for (PathElement element : pathElements) {
            if (element instanceof Waypoint) {
                ResolvedConstraintValue translationVelocity = resolveConstraintValue(
                    "translation velocity",
                    translationOrdinal,
                    pathConstraints.getMaxVelocityMetersPerSec(),
                    pathConstraints.getMinVelocityMetersPerSec(),
                    defaultGlobalConstraints.getMaxVelocityMetersPerSec(), reportWarnings
                );
                double translationAcceleration = resolveMaxConstraintValue(
                    pathConstraints.getMaxAccelerationMetersPerSec2(),
                    translationOrdinal,
                    defaultGlobalConstraints.getMaxAccelerationMetersPerSec2()
                );
                ResolvedConstraintValue rotationVelocity = resolveConstraintValue(
                    "rotation velocity",
                    rotationOrdinal,
                    pathConstraints.getMaxVelocityDegPerSec(),
                    pathConstraints.getMinVelocityDegPerSec(),
                    defaultGlobalConstraints.getMaxVelocityDegPerSec(), reportWarnings
                );
                double rotationAcceleration = resolveMaxConstraintValue(
                    pathConstraints.getMaxAccelerationDegPerSec2(),
                    rotationOrdinal,
                    defaultGlobalConstraints.getMaxAccelerationDegPerSec2()
                );

                elementsWithConstraints.add(
                    new Pair<>(
                        element,
                        new WaypointConstraint(
                            translationVelocity.maxValue(),
                            translationAcceleration,
                            rotationVelocity.maxValue(),
                            rotationAcceleration,
                            translationVelocity.minValue(),
                            rotationVelocity.minValue()
                        )
                    )
                );
                translationOrdinal++;
                rotationOrdinal++;
            }
            else if (element instanceof TranslationTarget) {
                ResolvedConstraintValue translationVelocity = resolveConstraintValue(
                    "translation velocity",
                    translationOrdinal,
                    pathConstraints.getMaxVelocityMetersPerSec(),
                    pathConstraints.getMinVelocityMetersPerSec(),
                    defaultGlobalConstraints.getMaxVelocityMetersPerSec(), reportWarnings
                );
                double translationAcceleration = resolveMaxConstraintValue(
                    pathConstraints.getMaxAccelerationMetersPerSec2(),
                    translationOrdinal,
                    defaultGlobalConstraints.getMaxAccelerationMetersPerSec2()
                );

                elementsWithConstraints.add(
                    new Pair<>(
                        element,
                        new TranslationTargetConstraint(
                            translationVelocity.maxValue(),
                            translationAcceleration,
                            translationVelocity.minValue()
                        )
                    )
                );
                translationOrdinal++;
            }
            else if (element instanceof RotationTarget) {
                ResolvedConstraintValue rotationVelocity = resolveConstraintValue(
                    "rotation velocity",
                    rotationOrdinal,
                    pathConstraints.getMaxVelocityDegPerSec(),
                    pathConstraints.getMinVelocityDegPerSec(),
                    defaultGlobalConstraints.getMaxVelocityDegPerSec(), reportWarnings
                );
                double rotationAcceleration = resolveMaxConstraintValue(
                    pathConstraints.getMaxAccelerationDegPerSec2(),
                    rotationOrdinal,
                    defaultGlobalConstraints.getMaxAccelerationDegPerSec2()
                );
                elementsWithConstraints.add(
                    new Pair<>(
                        element,
                        new RotationTargetConstraint(
                            rotationVelocity.maxValue(),
                            rotationAcceleration,
                            rotationVelocity.minValue()
                        )
                    )
                );

                rotationOrdinal++;
            } else if (element instanceof EventTrigger) {
                elementsWithConstraints.add(new Pair<>(element, null));
            }
        }

        return elementsWithConstraints;
    }

    /**
     * Gets all path elements with constraints, expanding waypoints into separate translation and rotation targets.
     * 
     * <p>This method is used by the path follower to get a flat list of targets without
     * waypoint wrappers. Waypoints are split into their component translation and rotation
     * targets with appropriate t_ratios set.
     * 
     * @return List of (PathElement, PathElementConstraint) pairs with waypoints expanded
     */
    List<Pair<PathElement, PathElementConstraint>> getPathElementsWithConstraintsNoWaypoints() {
        return getPathElementsWithConstraintsNoWaypoints(defaultState.constraints(), true);
    }

    List<Pair<PathElement, PathElementConstraint>> getPathElementsWithConstraintsNoWaypoints(
        DefaultGlobalConstraints defaults, boolean reportWarnings
    ) {
        if (elementValidationError().isPresent()) {
            return new ArrayList<>();
        }
        
        List<Pair<PathElement, PathElementConstraint>> elementsWithConstraints = getPathElementsWithConstraints(defaults, reportWarnings);
        List<Pair<PathElement, PathElementConstraint>> out = new ArrayList<>();
        for (int i = 0; i < elementsWithConstraints.size(); i++) {
            PathElement element = elementsWithConstraints.get(i).getFirst();
            PathElementConstraint constraint = elementsWithConstraints.get(i).getSecond();
            if (element instanceof Waypoint) {
                TranslationTarget translationTarget = ((Waypoint)element).translationTarget();
                TranslationTargetConstraint translationTargetConstraint = new TranslationTargetConstraint(
                    ((WaypointConstraint)constraint).maxVelocityMetersPerSec(), 
                    ((WaypointConstraint)constraint).maxAccelerationMetersPerSec2(),
                    ((WaypointConstraint)constraint).minVelocityMetersPerSec()
                );
                RotationTarget rotationTarget = ((Waypoint)element).rotationTarget();
                RotationTargetConstraint rotationTargetConstraint = new RotationTargetConstraint(
                    ((WaypointConstraint)constraint).maxVelocityDegPerSec(), 
                    ((WaypointConstraint)constraint).maxAccelerationDegPerSec2(),
                    ((WaypointConstraint)constraint).minVelocityDegPerSec()
                );
                if (i == 0 && hasAuthoredStart()) {
                    rotationTarget = new RotationTarget(
                        rotationTarget.rotation(), 
                        0, 
                        rotationTarget.profiledRotation()
                    );

                    out.add(new Pair<>(
                        translationTarget, 
                        translationTargetConstraint
                    ));
                    out.add(new Pair<>(
                        rotationTarget, 
                        rotationTargetConstraint
                    ));
                } else {
                    rotationTarget = new RotationTarget(
                        rotationTarget.rotation(), 
                        1, 
                        rotationTarget.profiledRotation()
                    );
                    out.add(new Pair<>(
                        rotationTarget, 
                        rotationTargetConstraint
                    ));
                    out.add(new Pair<>(
                        translationTarget, 
                        translationTargetConstraint
                    ));
                }
            } else {
                out.add(elementsWithConstraints.get(i));
            }
        }
        return out;
    }

    /**
     * Selects the desired alliance flip state. Repeating the same value has no effect.
     * The transform is a half-turn around the field center; authored element order is preserved.
     * @param flipped whether to use the opposite alliance's coordinates
     * @return this path
     */
    public Path setFlipped(boolean flipped) {
        if (this.flipped != flipped) {
            transformElements(
                t -> new Translation2d(FlippingUtil.fieldSizeX - t.getX(), FlippingUtil.fieldSizeY - t.getY()),
                r -> r.minus(Rotation2d.PI));
            this.flipped = flipped;
        }
        return this;
    }

    /**
     * Selects the desired reflection state across the field width. Repeating a value has no effect.
     * @param mirrored whether to reflect Y and negate headings
     * @return this path
     */
    public Path setMirrored(boolean mirrored) {
        if (this.mirrored != mirrored) {
            transformElements(FlippingUtil::mirrorFieldPosition, FlippingUtil::mirrorFieldRotation);
            this.mirrored = mirrored;
        }
        return this;
    }

    /** @return whether alliance flipping is currently applied */
    public boolean isFlipped() { return flipped; }

    /** @return whether field-width reflection is currently applied */
    public boolean isMirrored() { return mirrored; }

    /** @deprecated Use {@link #setFlipped(boolean)} to state the desired result explicitly. */
    @Deprecated
    public void flip() { setFlipped(true); }

    /** @deprecated Use {@link #setFlipped(boolean)} with false. */
    @Deprecated
    public void undoFlip() { setFlipped(false); }

    /** @deprecated This toggles reflection. Use {@link #setMirrored(boolean)} for desired state. */
    @Deprecated
    public void mirror() { setMirrored(!mirrored); }

    private void transformElements(
        java.util.function.UnaryOperator<Translation2d> position,
        java.util.function.UnaryOperator<Rotation2d> heading
    ) {
        for (int i = 0; i < pathElements.size(); i++) {
            PathElement element = pathElements.get(i);
            if (element instanceof TranslationTarget t) {
                pathElements.set(i, new TranslationTarget(position.apply(t.translation()), t.intermediateHandoffRadiusMeters(), t.handoffMode()));
            } else if (element instanceof RotationTarget r) {
                pathElements.set(i, new RotationTarget(heading.apply(r.rotation()), r.t_ratio(), r.profiledRotation()));
            } else if (element instanceof Waypoint w) {
                TranslationTarget t = w.translationTarget();
                RotationTarget r = w.rotationTarget();
                pathElements.set(i, new Waypoint(
                    new TranslationTarget(position.apply(t.translation()), t.intermediateHandoffRadiusMeters(), t.handoffMode()),
                    new RotationTarget(heading.apply(r.rotation()), r.t_ratio(), r.profiledRotation())));
            }
        }
    }

    /**
     * Returns the authored starting pose, using zero heading for a translation-only start.
     * A single destination or a path beginning with a rotation/event has no authored start.
     *
     * <pre>{@code
     * path.getAuthoredStartPose().ifPresent(drive::resetPose);
     * }</pre>
     *
     * @return the authored start, or empty when execution must start from the measured pose
     * @throws IllegalStateException if the path's elements are invalid
     * @see #getAuthoredStartPose(Rotation2d)
     */
    public Optional<Pose2d> getAuthoredStartPose() {
        return getAuthoredStartPose(new Rotation2d());
    }

    /**
     * Returns the authored starting pose in this path's current flip/mirror state.
     * Only a starting waypoint supplies an authored heading. A translation-only start uses
     * {@code fallbackRotation}; a later rotation target is a destination, not a reset heading.
     * Motion constraints are not checked by this geometry query.
     *
     * @param fallbackRotation heading to use for an authored translation-only start
     * @return the authored start, or empty for a current-pose start
     * @throws IllegalStateException if the path's elements are invalid
     */
    public Optional<Pose2d> getAuthoredStartPose(Rotation2d fallbackRotation) {
        java.util.Objects.requireNonNull(fallbackRotation, "fallbackRotation");
        elementValidationError().ifPresent(message -> {
            throw new IllegalStateException("Cannot compute authored start: " + message);
        });
        if (!hasAuthoredStart()) return Optional.empty();
        PathElement first = pathElements.getFirst();
        if (first instanceof Waypoint waypoint) {
            return Optional.of(new Pose2d(waypoint.translationTarget().translation(), waypoint.rotationTarget().rotation()));
        }
        return Optional.of(new Pose2d(((TranslationTarget) first).translation(), fallbackRotation));
    }

    /**
     * Returns the authored starting pose, using zero heading for a translation-only start.
     * Use {@link #getAuthoredStartPose()} when a path may instead start from the measured pose.
     *
     * @return the authored starting pose
     * @throws IllegalStateException if the elements are invalid or there is no authored start
     */
    public Pose2d getStartPose() {
        return getStartPose(new Rotation2d());
    }

    /**
     * Returns the authored starting pose with a fallback heading for a translation-only start.
     * This method never returns a single destination as the starting pose.
     *
     * @param fallbackRotation heading to use for an authored translation-only start
     * @return the authored starting pose
     * @throws IllegalStateException if the elements are invalid or there is no authored start
     */
    public Pose2d getStartPose(Rotation2d fallbackRotation) {
        return getAuthoredStartPose(fallbackRotation).orElseThrow(() ->
            new IllegalStateException("Path has no authored start; use getAuthoredStartPose() or the robot's measured pose"));
    }

    /**
     * Gets the initial direction the robot's modules should face when starting this path.
     * 
     * <p><b>Recommendation:</b> It is highly recommended to pre-orient swerve modules toward
     * this direction before the start of an autonomous routine (either via a command or by
     * physically setting module orientations during robot setup). This prevents micro-deviations
     * at the start of the autonomous routine caused by modules needing to rotate before driving.
     * 
     * <p>This optimization is primarily important for the autonomous phase where precise initial
     * movement matters most. 
     * 
     * @throws IllegalStateException if the path has no authored start; use the pose-supplier overload
     * @return The initial module direction as a Rotation2d
     * @see #getStartPose()
     */
    public Rotation2d getInitialModuleDirection() {
        return getInitialModuleDirection(this::getStartPose);
    }

    /**
     * Gets the initial module direction using a fallback rotation for the start pose.
     * 
     * <p><b>Recommendation:</b> It is highly recommended to pre-orient swerve modules toward
     * this direction before the start of an autonomous routine. See {@link #getInitialModuleDirection()}
     * for details.
     * 
     * @throws IllegalStateException if the path has no authored start; use the pose-supplier overload
     * @param fallbackRotation The fallback rotation for computing start pose
     * @return The initial module direction
     * @see #getInitialModuleDirection()
     */
    public Rotation2d getInitialModuleDirection(Rotation2d fallbackRotation) {
        return getInitialModuleDirection(() -> getStartPose(fallbackRotation));
    }

    /**
     * Gets the initial module direction using a custom pose supplier.
     * 
     * <p>This calculates the direction the swerve modules should face when beginning
     * to follow the path, pointing toward the first translation target that is outside
     * the robot's current handoff radius.
     * 
     * <p><b>Recommendation:</b> It is highly recommended to pre-orient swerve modules toward
     * this direction before the start of an autonomous routine (either via a command or by
     * physically setting module orientations during robot setup). This prevents micro-deviations
     * at the start of the autonomous routine caused by modules needing to rotate before driving.
     * 
     * <p>This optimization is primarily important for the autonomous phase where precise initial
     * movement matters most. 
     * 
     * @param poseSupplier Supplier for the robot's current pose
     * @return The initial module direction relative to the robot's rotation
     * @see #getInitialModuleDirection()
     */
    public Rotation2d getInitialModuleDirection(Supplier<Pose2d> poseSupplier) {
        Pose2d robotPose = poseSupplier.get();
        
        if (elementValidationError().isPresent()) {
            return new Rotation2d(0);
        }

        // Get all translation targets from the path
        List<Pair<PathElement, PathElementConstraint>> pathElements = getPathElementsWithConstraintsNoWaypoints();
        List<TranslationTarget> translationTargets = new ArrayList<>();
        for (Pair<PathElement, PathElementConstraint> element : pathElements) {
            if (element.getFirst() instanceof TranslationTarget) {
                translationTargets.add((TranslationTarget) element.getFirst());
            }
        }

        if (translationTargets.isEmpty()) {
            return new Rotation2d(0);
        }

        // if there is one translation target, return direction of translation target from robot pose
        if (translationTargets.size() == 1) {
            Translation2d target = translationTargets.get(0).translation();
            return 
                robotPose.getRotation().minus(new Rotation2d(
                    target.getX() - robotPose.getTranslation().getX(),
                    target.getY() - robotPose.getTranslation().getY()
                ));
        }

        // if there is more than one translation target, choose the first target which's handoff radius does not intersect robot pose and return direction of that target from robot pose
        for (TranslationTarget target : translationTargets) {
            double handoffRadius = target.intermediateHandoffRadiusMeters()
                .orElse(getDefaultGlobalConstraints().getIntermediateHandoffRadiusMeters());
            double distanceToTarget = robotPose.getTranslation().getDistance(target.translation());

            // If handoff radius exists and robot pose is outside the handoff radius, use this target
            if (distanceToTarget > handoffRadius) {
                return 
                    robotPose.getRotation().minus(new Rotation2d(
                        target.translation().getX() - robotPose.getTranslation().getX(),
                        target.translation().getY() - robotPose.getTranslation().getY()
                    ));
            }
        }

        // if there is no target which's handoff radius does not intersect robot pose, return direction of the last target from robot pose
        Translation2d lastTarget = translationTargets.get(translationTargets.size() - 1).translation();
        return 
            robotPose.getRotation().minus(new Rotation2d(
                lastTarget.getX() - robotPose.getTranslation().getX(),
                lastTarget.getY() - robotPose.getTranslation().getY()
            ));
    }
    
    private Path(Path source, List<PathElement> elements) {
        pathElements = elements;
        pathConstraints = source.pathConstraints.copy();
        flipped = source.flipped;
        mirrored = source.mirrored;
        handoffMode = source.handoffMode;
        tankDriveDirection = source.tankDriveDirection;
    }

    /**
     * Creates a deep copy of this path.
     * 
     * <p>The copy includes all path elements, constraints, and preserves both transform states.
     * 
     * @return A new Path with copied data
     */
    public Path copy() {
        List<PathElement> deepCopiedElements = new ArrayList<>(pathElements.size());
        for (PathElement element : pathElements) {
            deepCopiedElements.add(element == null ? null : element.copy());
        }
        return new Path(this, deepCopiedElements);
    }
}
