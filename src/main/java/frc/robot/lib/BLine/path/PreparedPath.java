package frc.robot.lib.BLine.path;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.util.Pair;

/**
 * Immutable execution data shared by the path loader and follower.
 * <p>Internal library bridge, not an application configuration API. Use {@link Path} and
 * the command builders; this type is public only to cross Java package boundaries.
 */
public record PreparedPath(
    Optional<AuthoredStart> authoredStart,
    DriveDirection direction,
    List<Pair<Path.PathElement, MotionConstraint>> elements,
    Defaults defaults,
    HandoffMode handoffMode,
    double translationTolerance,
    double rotationToleranceDegrees,
    boolean rollingEnd,
    OptionalDouble tankFinalHeading
) {
    public PreparedPath {
        elements = List.copyOf(elements);
    }

    public record AuthoredStart(Translation2d translation, Optional<Rotation2d> rotation) {
        public Pose2d pose(Rotation2d measuredHeading) { return new Pose2d(translation, rotation.orElse(measuredHeading)); }
    }
    public record Defaults(double maxVelocityMetersPerSec, double maxAccelerationMetersPerSec2,
                           double maxVelocityDegPerSec, double maxAccelerationDegPerSec2) {}
    public sealed interface MotionConstraint permits TranslationLimits, RotationLimits {}
    public record TranslationLimits(double maxVelocityMetersPerSec, double maxAccelerationMetersPerSec2,
                                    double minVelocityMetersPerSec) implements MotionConstraint {}
    public record RotationLimits(double maxVelocityDegPerSec, double maxAccelerationDegPerSec2,
                                 double minVelocityDegPerSec) implements MotionConstraint {
        public RotationLimits(double speed, double acceleration) { this(speed, acceleration, 0); }
    }

    public static PreparedPath create(Path source, Optional<Boolean> flipped, Optional<Boolean> mirrored) {
        return create(source, flipped, mirrored, true);
    }

    static Optional<String> validationError(Path source) {
        try {
            create(source, Optional.empty(), Optional.empty(), false);
            return Optional.empty();
        } catch (IllegalArgumentException e) {
            return Optional.of(e.getMessage());
        }
    }

    private static PreparedPath create(Path source, Optional<Boolean> flipped, Optional<Boolean> mirrored,
                                       boolean reportWarnings) {
        source.elementValidationError().ifPresent(message -> { throw new IllegalArgumentException(message); });
        Path path = source.copy();
        flipped.ifPresent(path::setFlipped);
        mirrored.ifPresent(path::setMirrored);
        ProjectDefaults settings = Path.currentProjectDefaults();
        Path.DefaultGlobalConstraints defaults = settings.constraints();
        positive(defaults.getMaxVelocityMetersPerSec(), "Default maximum translation velocity");
        positive(defaults.getMaxAccelerationMetersPerSec2(), "Default translation acceleration");
        positive(defaults.getMaxVelocityDegPerSec(), "Default maximum angular velocity");
        positive(defaults.getMaxAccelerationDegPerSec2(), "Default angular acceleration");
        nonnegative(defaults.getIntermediateHandoffRadiusMeters(), "Default handoff distance");
        double translationTolerance = path.getPathConstraints().getEndTranslationToleranceMeters().orElse(defaults.getEndTranslationToleranceMeters());
        double rotationTolerance = path.getPathConstraints().getEndRotationToleranceDeg().orElse(defaults.getEndRotationToleranceDeg());
        positive(translationTolerance, "End translation tolerance");
        positive(rotationTolerance, "End rotation tolerance");
        HandoffMode mode = path.getHandoffMode().orElse(settings.handoffMode());
        List<Pair<Path.PathElement, Path.PathElementConstraint>> elements = path.getPathElementsWithConstraintsNoWaypoints(defaults, reportWarnings).stream().map(entry -> {
            if (entry.getFirst() instanceof Path.TranslationTarget target) {
                Path.PathElement resolved = new Path.TranslationTarget(target.translation(),
                    Optional.of(target.intermediateHandoffRadiusMeters().orElse(defaults.getIntermediateHandoffRadiusMeters())),
                    Optional.of(target.handoffMode().orElse(mode)));
                return new Pair<>(resolved, entry.getSecond());
            }
            return entry;
        }).toList();
        int translationOrdinal = 0, rotationOrdinal = 0;
        for (var entry : elements) {
            if (entry.getSecond() instanceof Path.TranslationTargetConstraint constraint) {
                String context = "Translation " + (++translationOrdinal) + ": ";
                positive(constraint.maxVelocityMetersPerSec(), context + "maximum velocity");
                positive(constraint.maxAccelerationMetersPerSec2(), context + "acceleration");
                nonnegative(constraint.minVelocityMetersPerSec(), context + "minimum velocity");
            } else if (entry.getSecond() instanceof Path.RotationTargetConstraint constraint) {
                String context = "Rotation " + (++rotationOrdinal) + ": ";
                positive(constraint.maxVelocityDegPerSec(), context + "maximum velocity");
                positive(constraint.maxAccelerationDegPerSec2(), context + "acceleration");
                nonnegative(constraint.minVelocityDegPerSec(), context + "minimum velocity");
            }
        }
        boolean rolling = ((Path.TranslationTargetConstraint) elements.getLast().getSecond()).minVelocityMetersPerSec() > 0;
        Path.PathElement last = path.getPathElements().getLast();
        OptionalDouble finalHeading = last instanceof Path.Waypoint waypoint
            ? OptionalDouble.of(waypoint.rotationTarget().rotation().getRadians()) : OptionalDouble.empty();
        List<Pair<Path.PathElement, MotionConstraint>> resolved = elements.stream().map(entry -> {
            MotionConstraint constraint = switch (entry.getSecond()) {
                case Path.TranslationTargetConstraint c -> new TranslationLimits(c.maxVelocityMetersPerSec(), c.maxAccelerationMetersPerSec2(), c.minVelocityMetersPerSec());
                case Path.RotationTargetConstraint c -> new RotationLimits(c.maxVelocityDegPerSec(), c.maxAccelerationDegPerSec2(), c.minVelocityDegPerSec());
                case null -> null; // Events have no motion constraints.
                default -> throw new IllegalStateException("Prepared waypoints must be expanded");
            };
            return new Pair<>(entry.getFirst(), constraint);
        }).toList();
        Optional<AuthoredStart> start = Optional.empty();
        if (path.hasAuthoredStart()) {
            Path.PathElement first = path.getPathElements().getFirst();
            start = Optional.of(first instanceof Path.Waypoint waypoint
                ? new AuthoredStart(waypoint.translationTarget().translation(), Optional.of(waypoint.rotationTarget().rotation()))
                : new AuthoredStart(((Path.TranslationTarget) first).translation(), Optional.empty()));
        }
        return new PreparedPath(start, path.getTankDriveDirection(), resolved,
            new Defaults(defaults.getMaxVelocityMetersPerSec(), defaults.getMaxAccelerationMetersPerSec2(),
                defaults.getMaxVelocityDegPerSec(), defaults.getMaxAccelerationDegPerSec2()),
            mode, translationTolerance, rotationTolerance, rolling, finalHeading);
    }

    private static void positive(double value, String label) {
        if (!Double.isFinite(value) || value <= 0) throw new IllegalArgumentException(label + " must be finite and positive");
    }

    private static void nonnegative(double value, String label) {
        if (!Double.isFinite(value) || value < 0) throw new IllegalArgumentException(label + " must be finite and nonnegative");
    }
}
