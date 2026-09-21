package frc.robot.lib.BLine;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.util.Pair;

/** Validated, resolved execution snapshot. Preparation has no robot or event side effects. */
record PreparedPath(
    Path path,
    List<Pair<Path.PathElement, Path.PathElementConstraint>> elements,
    Path.DefaultGlobalConstraints defaults,
    HandoffMode handoffMode,
    double translationTolerance,
    double rotationToleranceDegrees,
    boolean rollingEnd,
    OptionalDouble tankFinalHeading
) {
    static PreparedPath create(Path source, Optional<Boolean> flipped, Optional<Boolean> mirrored) {
        source.validationError().ifPresent(message -> { throw new IllegalArgumentException(message); });
        Path path = source.copy();
        flipped.ifPresent(path::setFlipped);
        mirrored.ifPresent(path::setMirrored);
        Path.DefaultGlobalConstraints defaults = path.getDefaultGlobalConstraints();
        positive(defaults.getMaxVelocityMetersPerSec(), "Default maximum translation velocity");
        positive(defaults.getMaxAccelerationMetersPerSec2(), "Default translation acceleration");
        positive(defaults.getMaxVelocityDegPerSec(), "Default maximum angular velocity");
        positive(defaults.getMaxAccelerationDegPerSec2(), "Default angular acceleration");
        nonnegative(defaults.getIntermediateHandoffRadiusMeters(), "Default handoff distance");
        double translationTolerance = path.getEndTranslationToleranceMeters();
        double rotationTolerance = path.getEndRotationToleranceDeg();
        positive(translationTolerance, "End translation tolerance");
        positive(rotationTolerance, "End rotation tolerance");
        HandoffMode mode = path.getHandoffMode().orElse(Path.getDefaultHandoffMode());
        List<Pair<Path.PathElement, Path.PathElementConstraint>> elements = path.getPathElementsWithConstraintsNoWaypoints().stream().map(entry -> {
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
        return new PreparedPath(path, elements, defaults, mode, translationTolerance, rotationTolerance, rolling, finalHeading);
    }

    private static void positive(double value, String label) {
        if (!Double.isFinite(value) || value <= 0) throw new IllegalArgumentException(label + " must be finite and positive");
    }

    private static void nonnegative(double value, String label) {
        if (!Double.isFinite(value) || value < 0) throw new IllegalArgumentException(label + " must be finite and nonnegative");
    }
}
