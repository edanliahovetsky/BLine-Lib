package frc.robot.lib.BLine;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.logging.Level;
import java.util.logging.Logger;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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
 * @see JsonUtils
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
    public sealed interface PathElement permits Waypoint, TranslationTarget, RotationTarget {
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
    public sealed interface PathElementConstraint permits WaypointConstraint, TranslationTargetConstraint, RotationTargetConstraint {}

    /**
     * Constraints for a {@link Waypoint}, including both translation and rotation limits.
     * 
     * @param maxVelocityMetersPerSec Maximum translational velocity in meters per second
     * @param maxAccelerationMetersPerSec2 Maximum translational acceleration in meters per second squared
     * @param maxVelocityDegPerSec Maximum rotational velocity in degrees per second
     * @param maxAccelerationDegPerSec2 Maximum rotational acceleration in degrees per second squared
     */
    public static record WaypointConstraint(
        double maxVelocityMetersPerSec,
        double maxAccelerationMetersPerSec2,
        double maxVelocityDegPerSec,
        double maxAccelerationDegPerSec2
    ) implements PathElementConstraint {}

    /**
     * Constraints for a {@link TranslationTarget}.
     * 
     * @param maxVelocityMetersPerSec Maximum translational velocity in meters per second
     * @param maxAccelerationMetersPerSec2 Maximum translational acceleration in meters per second squared
     */
    public static record TranslationTargetConstraint(
        double maxVelocityMetersPerSec,
        double maxAccelerationMetersPerSec2
    ) implements PathElementConstraint {}

    /**
     * Constraints for a {@link RotationTarget}.
     * 
     * @param maxVelocityDegPerSec Maximum rotational velocity in degrees per second
     * @param maxAccelerationDegPerSec2 Maximum rotational acceleration in degrees per second squared
     */
    public static record RotationTargetConstraint(
        double maxVelocityDegPerSec,
        double maxAccelerationDegPerSec2
    ) implements PathElementConstraint {}

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
        /**
         * Creates a deep copy of this waypoint.
         * 
         * @return A new Waypoint with copied translation and rotation targets
         */
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
        Optional<Double> intermediateHandoffRadiusMeters
    ) implements PathElement {
        /**
         * Creates a copy of this translation target.
         * 
         * @return A new TranslationTarget with the same values
         */
        public TranslationTarget copy() {
            return new TranslationTarget(translation, intermediateHandoffRadiusMeters);
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
        
        /** @return Optional end translation tolerance override */
        public Optional<Double> getEndTranslationToleranceMeters() { return endTranslationToleranceMeters; }
        
        /** @return Optional end rotation tolerance override */
        public Optional<Double> getEndRotationToleranceDeg() { return endRotationToleranceDeg; }

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
            if (endTranslationToleranceMeters.isPresent()) c.setEndTranslationToleranceMeters(endTranslationToleranceMeters.get());
            if (endRotationToleranceDeg.isPresent()) c.setEndRotationToleranceDeg(endRotationToleranceDeg.get());
            return c;
        }
    }


    private List<PathElement> pathElements;
    private PathConstraints pathConstraints;
    private static DefaultGlobalConstraints defaultGlobalConstraints = null;
    private boolean flipped = false;
    private boolean isValid = true;
    
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
        if (pathElements == null) {
            throw new IllegalArgumentException("pathElements cannot be null");
        }
        if (constraints == null) {
            constraints = new PathConstraints();
        }
        if (defaultGlobalConstraints == null) {
            try {
                defaultGlobalConstraints = JsonUtils.loadGlobalConstraints(JsonUtils.PROJECT_ROOT);
            } catch (RuntimeException e) {
                // Allow defaultGlobalConstraints to remain null if loading fails
                throw new RuntimeException("Failed to load default global constraints", e);
            }
        }
        
        this.pathElements = new ArrayList<>(pathElements);
        this.pathConstraints = constraints.copy();
        if (defaultGlobalConstraints != null) {
            Path.defaultGlobalConstraints = defaultGlobalConstraints.copy();
        }
        
        // Validate that first and last elements are both either waypoints or translation targets
        validatePathEndpoints();
    }

    /**
     * Creates a new Path from varargs path elements using current global defaults.
     * 
     * @param pathElements The path elements defining the path
     */
    public Path(PathElement... pathElements) {
        this(List.of(pathElements), null, Path.defaultGlobalConstraints);
    }

    /**
     * Creates a new Path with custom constraints from varargs path elements.
     * 
     * @param constraints The path-specific constraints
     * @param pathElements The path elements defining the path
     */
    public Path(PathConstraints constraints, PathElement... pathElements) {
        this(List.of(pathElements), constraints, Path.defaultGlobalConstraints);
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
        this(pathElements, null, Path.defaultGlobalConstraints);
    }

    /**
     * Creates a new Path from a list with custom constraints.
     * 
     * @param pathElements The list of path elements
     * @param constraints The path-specific constraints
     */
    public Path(List<PathElement> pathElements, PathConstraints constraints) {
        this(pathElements, constraints, Path.defaultGlobalConstraints);
    }

    /**
     * Loads a Path from a JSON file in the specified autos directory.
     * 
     * @param autosDir The directory containing the autos folder structure
     * @param pathFileName The name of the path file (without .json extension)
     */
    public Path(File autosDir, String pathFileName) {
        Path loaded = JsonUtils.loadPath(autosDir, pathFileName+".json");
        this.pathElements = loaded.pathElements;
        this.pathConstraints = loaded.pathConstraints;
        // globals are static and already copied

        // Validate that first and last elements are both either waypoints or translation targets
        validatePathEndpoints();
    }

    /**
     * Loads a Path from a JSON file in the default project root directory.
     * 
     * <p>The file is loaded from {@code deploy/autos/paths/<pathFileName>.json}.
     * 
     * @param pathFileName The name of the path file (without .json extension)
     */
    public Path(String pathFileName) {
        this(JsonUtils.PROJECT_ROOT, pathFileName);
    }

    /**
     * Validates that the first and last path elements are both either waypoints or translation targets.
     * If the path has only 1 element, it must be a waypoint or translation target.
     * Logs an error if validation fails but allows program execution to continue.
     */
    private void validatePathEndpoints() {
        if (pathElements.size() == 0) {
            isValid = false;
            logger.log(Level.WARNING, "Path validation failed: Path cannot be empty");
            return;
        }

        if (pathElements.size() == 1) {
            PathElement element = pathElements.get(0);
            if (element instanceof RotationTarget) {
                isValid = false;
                logger.log(Level.WARNING, "Path validation failed: Path cannot consist of a single rotation target");
            }
            return;
        }

        PathElement first = pathElements.get(0);
        PathElement last = pathElements.get(pathElements.size() - 1);

        boolean firstIsValid = first instanceof Waypoint || first instanceof TranslationTarget;
        boolean lastIsValid = last instanceof Waypoint || last instanceof TranslationTarget;

        if (!firstIsValid || !lastIsValid) {
            isValid = false;
            logger.log(Level.WARNING, "Path validation failed: First and last path elements must both be either Waypoints or TranslationTargets. " +
                "First element is: " + first.getClass().getSimpleName() + ", " +
                "Last element is: " + last.getClass().getSimpleName());
        }
    }

    /**
     * Checks if this path passed validation.
     * 
     * @return true if the path is valid and can be followed, false otherwise
     */
    public boolean isValid() {
        return isValid;
    }

    /**
     * Gets a copy of the current default global constraints.
     * 
     * @return A copy of the default global constraints
     */
    public DefaultGlobalConstraints getDefaultGlobalConstraints() {
        return defaultGlobalConstraints.copy();
    }

    /**
     * Sets the default global constraints used by all paths.
     * 
     * <p>This is a static method that affects all subsequently created paths.
     * 
     * @param defaultGlobalConstraints The new global constraints
     * @throws IllegalArgumentException if defaultGlobalConstraints is null
     */
    public static void setDefaultGlobalConstraints(DefaultGlobalConstraints defaultGlobalConstraints) {
        if (defaultGlobalConstraints == null) {
            throw new IllegalArgumentException("defaultGlobalConstraints cannot be null");
        }
        Path.defaultGlobalConstraints = defaultGlobalConstraints.copy();
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
        return pathConstraints.getEndTranslationToleranceMeters().orElse(defaultGlobalConstraints.getEndTranslationToleranceMeters());
    }

    /**
     * Gets the end rotation tolerance, using path-specific if set, otherwise global default.
     * 
     * @return The end rotation tolerance in degrees
     */
    public double getEndRotationToleranceDeg() {
        return pathConstraints.getEndRotationToleranceDeg().orElse(defaultGlobalConstraints.getEndRotationToleranceDeg());
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
     * @throws IllegalArgumentException if newOrder doesn't match the number of elements
     */
    public Path reorderElements(List<Integer> newOrder) {
        if (newOrder.size() != pathElements.size()) {
            throw new IllegalArgumentException("New order must match elements length");
        }
        List<PathElement> reordered = new ArrayList<>(pathElements.size());
        for (int i : newOrder) {
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

    /**
     * Gets all path elements paired with their computed constraints.
     * 
     * <p>This method resolves which constraints apply to each element based on
     * path-specific constraints and global defaults.
     * 
     * @return List of (PathElement, PathElementConstraint) pairs
     */
    public List<Pair<PathElement, PathElementConstraint>> getPathElementsWithConstraints() {
        if (!isValid()) {
            return new ArrayList<>();
        }
        
        List<Pair<PathElement, PathElementConstraint>> elementsWithConstraints = new ArrayList<>();
        int translationOrdinal = 0;
        int rotationOrdinal = 0;
        for (PathElement element : pathElements) {
            if (element instanceof Waypoint) {
                double maxVelocityMetersPerSec = -1;
                double maxAccelerationMetersPerSec2 = -1;
                double maxVelocityDegPerSec = -1;
                double maxAccelerationDegPerSec2 = -1;
                if (pathConstraints.getMaxVelocityMetersPerSec().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxVelocityMetersPerSec().get()) {
                        if (constraint.startOrdinal() <= translationOrdinal && constraint.endOrdinal() >= translationOrdinal) {
                            maxVelocityMetersPerSec = constraint.value();
                            break;
                        }
                    }
                }
                if (pathConstraints.getMaxAccelerationMetersPerSec2().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxAccelerationMetersPerSec2().get()) {
                        if (constraint.startOrdinal() <= translationOrdinal && constraint.endOrdinal() >= translationOrdinal) {
                            maxAccelerationMetersPerSec2 = constraint.value();
                            break;
                        }
                    }
                }
                if (pathConstraints.getMaxVelocityDegPerSec().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxVelocityDegPerSec().get()) {
                        if (constraint.startOrdinal() <= rotationOrdinal && constraint.endOrdinal() >= rotationOrdinal) {
                            maxVelocityDegPerSec = constraint.value();
                            break;
                        }
                    }
                }
                if (pathConstraints.getMaxAccelerationDegPerSec2().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxAccelerationDegPerSec2().get()) {
                        if (constraint.startOrdinal() <= rotationOrdinal && constraint.endOrdinal() >= rotationOrdinal) {
                            maxAccelerationDegPerSec2 = constraint.value();
                            break;
                        }
                    }
                }
                maxVelocityMetersPerSec = maxVelocityMetersPerSec == -1 ? 
                    defaultGlobalConstraints.getMaxVelocityMetersPerSec() : maxVelocityMetersPerSec;
                maxAccelerationMetersPerSec2 = maxAccelerationMetersPerSec2 == -1 ? 
                    defaultGlobalConstraints.getMaxAccelerationMetersPerSec2() : maxAccelerationMetersPerSec2;
                maxVelocityDegPerSec = maxVelocityDegPerSec == -1 ? 
                    defaultGlobalConstraints.getMaxVelocityDegPerSec() : maxVelocityDegPerSec;
                maxAccelerationDegPerSec2 = maxAccelerationDegPerSec2 == -1 ? 
                    defaultGlobalConstraints.getMaxAccelerationDegPerSec2() : maxAccelerationDegPerSec2;

                elementsWithConstraints.add(
                    new Pair<>(
                        element, 
                        new WaypointConstraint(
                            maxVelocityMetersPerSec, 
                            maxAccelerationMetersPerSec2, 
                            maxVelocityDegPerSec, 
                            maxAccelerationDegPerSec2
                        )
                    )
                );
                translationOrdinal++;
                rotationOrdinal++;
            }
            else if (element instanceof TranslationTarget) {
                double maxVelocityMetersPerSec = -1;
                double maxAccelerationMetersPerSec2 = -1;
                if (pathConstraints.getMaxVelocityMetersPerSec().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxVelocityMetersPerSec().get()) {
                        if (constraint.startOrdinal() <= translationOrdinal && constraint.endOrdinal() >= translationOrdinal) {
                            maxVelocityMetersPerSec = constraint.value();
                            break;
                        }
                    }
                }
                if (pathConstraints.getMaxAccelerationMetersPerSec2().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxAccelerationMetersPerSec2().get()) {
                        if (constraint.startOrdinal() <= translationOrdinal && constraint.endOrdinal() >= translationOrdinal) {
                            maxAccelerationMetersPerSec2 = constraint.value();
                            break;
                        }
                    }
                }
                maxVelocityMetersPerSec = maxVelocityMetersPerSec == -1 ? 
                    defaultGlobalConstraints.getMaxVelocityMetersPerSec() : maxVelocityMetersPerSec;
                maxAccelerationMetersPerSec2 = maxAccelerationMetersPerSec2 == -1 ? 
                    defaultGlobalConstraints.getMaxAccelerationMetersPerSec2() : maxAccelerationMetersPerSec2;

                elementsWithConstraints.add(
                    new Pair<>(
                        element, 
                        new TranslationTargetConstraint(
                            maxVelocityMetersPerSec, 
                            maxAccelerationMetersPerSec2
                        )
                    )
                );
                translationOrdinal++;
            }
            else if (element instanceof RotationTarget) {
                double maxVelocityDegPerSec = -1;
                double maxAccelerationDegPerSec2 = -1;
                if (pathConstraints.getMaxVelocityDegPerSec().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxVelocityDegPerSec().get()) {
                        if (constraint.startOrdinal() <= rotationOrdinal && constraint.endOrdinal() >= rotationOrdinal) {
                            maxVelocityDegPerSec = constraint.value();
                            break;
                        }
                    }
                }
                if (pathConstraints.getMaxAccelerationDegPerSec2().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxAccelerationDegPerSec2().get()) {
                        if (constraint.startOrdinal() <= rotationOrdinal && constraint.endOrdinal() >= rotationOrdinal) {
                            maxAccelerationDegPerSec2 = constraint.value();
                            break;
                        }
                    }
                }
                maxVelocityDegPerSec = maxVelocityDegPerSec == -1 ? 
                    defaultGlobalConstraints.getMaxVelocityDegPerSec() : maxVelocityDegPerSec;
                maxAccelerationDegPerSec2 = maxAccelerationDegPerSec2 == -1 ? 
                    defaultGlobalConstraints.getMaxAccelerationDegPerSec2() : maxAccelerationDegPerSec2;
                elementsWithConstraints.add(new Pair<>(element, new RotationTargetConstraint(maxVelocityDegPerSec, maxAccelerationDegPerSec2)));

                rotationOrdinal++;
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
    public List<Pair<PathElement, PathElementConstraint>> getPathElementsWithConstraintsNoWaypoints() {
        if (!isValid()) {
            return new ArrayList<>();
        }
        
        List<Pair<PathElement, PathElementConstraint>> elementsWithConstraints = getPathElementsWithConstraints();
        List<Pair<PathElement, PathElementConstraint>> out = new ArrayList<>();
        for (int i = 0; i < elementsWithConstraints.size(); i++) {
            PathElement element = elementsWithConstraints.get(i).getFirst();
            PathElementConstraint constraint = elementsWithConstraints.get(i).getSecond();
            if (element instanceof Waypoint) {
                TranslationTarget translationTarget = ((Waypoint)element).translationTarget();
                TranslationTargetConstraint translationTargetConstraint = new TranslationTargetConstraint(
                    ((WaypointConstraint)constraint).maxVelocityMetersPerSec(), 
                    ((WaypointConstraint)constraint).maxAccelerationMetersPerSec2()
                );
                RotationTarget rotationTarget = ((Waypoint)element).rotationTarget();
                RotationTargetConstraint rotationTargetConstraint = new RotationTargetConstraint(
                    ((WaypointConstraint)constraint).maxVelocityDegPerSec(), 
                    ((WaypointConstraint)constraint).maxAccelerationDegPerSec2()
                );
                if (i == 0) {
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
     * Flips this path to the opposite alliance side.
     * 
     * <p>Uses {@link FlippingUtil} to transform all coordinates. This method only
     * flips once - subsequent calls have no effect until {@link #undoFlip()} is called.
     */
    public void flip() {
        if (!isValid()) {
            return;
        }
        
        if (flipped) return;

        for (int i = 0; i < pathElements.size(); i++) {
            PathElement element = pathElements.get(i);
            if (element instanceof TranslationTarget) {
                pathElements.set(i, new TranslationTarget(
                    FlippingUtil.flipFieldPosition(((TranslationTarget) element).translation()),
                    ((TranslationTarget) element).intermediateHandoffRadiusMeters()
                ));
            } else if (element instanceof RotationTarget) {
                pathElements.set(i, new RotationTarget(
                    FlippingUtil.flipFieldRotation(((RotationTarget) element).rotation()),
                    ((RotationTarget) element).t_ratio(),
                    ((RotationTarget) element).profiledRotation()
                ));
            } else if (element instanceof Waypoint) {
                pathElements.set(i, new Waypoint(
                    new TranslationTarget(
                        FlippingUtil.flipFieldPosition(((Waypoint) element).translationTarget().translation()),
                        ((Waypoint) element).translationTarget().intermediateHandoffRadiusMeters()
                    ),
                    new RotationTarget(
                        FlippingUtil.flipFieldRotation(((Waypoint) element).rotationTarget().rotation()),
                        ((Waypoint) element).rotationTarget().t_ratio(),
                        ((Waypoint) element).rotationTarget().profiledRotation()
                    )
                ));
            }
        }

        flipped = true;
    }

    /**
     * Undoes a previous flip operation, restoring original coordinates.
     * 
     * <p>Has no effect if the path has not been flipped.
     */
    public void undoFlip() {
        if (!isValid()) {
            return;
        }
        
        if (!flipped) return;
        flipped = false;
        flip();
        flipped = false;
    }

    /**
     * Gets the starting pose for this path using a default rotation of 0.
     * 
     * @return The starting pose
     * @throws IllegalStateException if the path is invalid or empty
     * @see #getInitialModuleDirection()
     */
    public Pose2d getStartPose() {
        return getStartPose(new Rotation2d());
    }

    /**
     * Gets the starting pose for this path.
     * 
     * <p>The translation comes from the first translation target. The rotation comes
     * from the first rotation target, or falls back to the provided rotation if none exists.
     * 
     * @param fallbackRotation The rotation to use if no rotation target is found
     * @return The starting pose
     * @throws IllegalStateException if the path is invalid or has no translation targets
     * @see #getInitialModuleDirection(Rotation2d)
     */
    public Pose2d getStartPose(Rotation2d fallbackRotation) {
        if (!isValid()) {
            throw new IllegalStateException("Path invalid - cannot compute start pose");
        }

        List<Pair<PathElement, PathElementConstraint>> elements = getPathElementsWithConstraintsNoWaypoints();
        if (elements.isEmpty()) {
            throw new IllegalStateException("Path must contain at least one element");
        }

        Translation2d resetTranslation = null;
        for (Pair<PathElement, PathElementConstraint> element : elements) {
            if (element.getFirst() instanceof TranslationTarget) {
                resetTranslation = ((TranslationTarget) element.getFirst()).translation();
                break;
            }
        }
        if (resetTranslation == null) {
            throw new IllegalStateException("Path must contain at least one translation target");
        }

        Rotation2d resetRotation = fallbackRotation;
        for (int i = 0; i < elements.size(); i++) {
            if (elements.get(i).getFirst() instanceof RotationTarget) {
                resetRotation = ((RotationTarget) elements.get(i).getFirst()).rotation();
                break;
            }
        }

        return new Pose2d(resetTranslation, resetRotation);
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
        
        if (!isValid()) {
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
    
    /**
     * Creates a deep copy of this path.
     * 
     * <p>The copy includes all path elements, constraints, and preserves the flipped state.
     * 
     * @return A new Path with copied data
     */
    public Path copy() {
        List<PathElement> deepCopiedElements = new ArrayList<>(pathElements.size());
        for (PathElement element : pathElements) {
            deepCopiedElements.add(element.copy());
        }
        Path copiedPath = new Path(deepCopiedElements, pathConstraints.copy(), defaultGlobalConstraints.copy());
        copiedPath.flipped = this.flipped;
        copiedPath.isValid = this.isValid;
        return copiedPath;
    }
}
