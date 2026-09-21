package frc.robot.lib.BLine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.util.Pair;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.PathElementConstraint;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import org.junit.jupiter.api.Test;

class PathConstraintsTest {
    private static final Path.DefaultGlobalConstraints GLOBALS =
        new Path.DefaultGlobalConstraints(5.0, 6.0, 700.0, 1400.0, 0.05, 2.0, 0.2);

    @Test
    void copyPreservesAllMinimumConstraintRanges() {
        Path.PathConstraints constraints = new Path.PathConstraints()
            .setMinVelocityMetersPerSec(new Path.RangedConstraint(0.7, 1, 2))
            .setMinVelocityDegPerSec(new Path.RangedConstraint(45.0, 3, 4));

        Path.PathConstraints copy = constraints.copy();

        assertEquals(0.7, copy.getMinVelocityMetersPerSec().orElseThrow().get(0).value(), 1e-9);
        assertEquals(1, copy.getMinVelocityMetersPerSec().orElseThrow().get(0).startOrdinal());
        assertEquals(2, copy.getMinVelocityMetersPerSec().orElseThrow().get(0).endOrdinal());
        assertEquals(45.0, copy.getMinVelocityDegPerSec().orElseThrow().get(0).value(), 1e-9);
    }

    @Test
    void rangedMinimumConstraintsResolveOnlyInsideTheirOrdinalRanges() {
        Path.PathConstraints constraints = new Path.PathConstraints()
            .setMaxVelocityMetersPerSec(new Path.RangedConstraint(3.0, 1, 1))
            .setMinVelocityMetersPerSec(new Path.RangedConstraint(0.9, 1, 1))
            .setMaxVelocityDegPerSec(new Path.RangedConstraint(300.0, 0, 0))
            .setMinVelocityDegPerSec(new Path.RangedConstraint(80.0, 0, 0));
        Path path = new Path(
            List.of(
                new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
                new Path.RotationTarget(Rotation2d.fromDegrees(90.0), 0.5),
                new Path.TranslationTarget(new Translation2d(1.0, 0.0)),
                new Path.TranslationTarget(new Translation2d(2.0, 0.0))
            ),
            constraints,
            GLOBALS
        );

        List<Pair<PathElement, PathElementConstraint>> resolved = path.getPathElementsWithConstraints();
        Path.TranslationTargetConstraint firstTranslation =
            (Path.TranslationTargetConstraint) resolved.get(0).getSecond();
        Path.RotationTargetConstraint rotation =
            (Path.RotationTargetConstraint) resolved.get(1).getSecond();
        Path.TranslationTargetConstraint secondTranslation =
            (Path.TranslationTargetConstraint) resolved.get(2).getSecond();
        Path.TranslationTargetConstraint thirdTranslation =
            (Path.TranslationTargetConstraint) resolved.get(3).getSecond();

        assertEquals(GLOBALS.getMaxVelocityMetersPerSec(), firstTranslation.maxVelocityMetersPerSec(), 1e-9);
        assertEquals(0.0, firstTranslation.minVelocityMetersPerSec(), 1e-9);
        assertEquals(300.0, rotation.maxVelocityDegPerSec(), 1e-9);
        assertEquals(80.0, rotation.minVelocityDegPerSec(), 1e-9);
        assertEquals(3.0, secondTranslation.maxVelocityMetersPerSec(), 1e-9);
        assertEquals(0.9, secondTranslation.minVelocityMetersPerSec(), 1e-9);
        assertEquals(GLOBALS.getMaxVelocityMetersPerSec(), thirdTranslation.maxVelocityMetersPerSec(), 1e-9);
        assertEquals(0.0, thirdTranslation.minVelocityMetersPerSec(), 1e-9);
    }

    @Test
    void minimumGreaterThanMaximumWarnsAndFallsBackToGlobalDefault() {
        AtomicBoolean sawWarning = new AtomicBoolean(false);
        Logger logger = Logger.getLogger(Path.class.getName());
        Handler handler = new Handler() {
            @Override
            public void publish(LogRecord record) {
                if (record.getLevel().intValue() >= Level.WARNING.intValue()
                    && record.getMessage().contains("Path constraint conflict")) {
                    sawWarning.set(true);
                }
            }

            @Override public void flush() {}
            @Override public void close() throws SecurityException {}
        };
        logger.addHandler(handler);
        try {
            Path.PathConstraints constraints = new Path.PathConstraints()
                .setMaxVelocityMetersPerSec(1.0)
                .setMinVelocityMetersPerSec(2.0);
            Path path = new Path(
                List.of(new Path.TranslationTarget(new Translation2d(1.0, 0.0))),
                constraints,
                GLOBALS
            );

            Path.TranslationTargetConstraint resolved =
                (Path.TranslationTargetConstraint) path.getPathElementsWithConstraints().get(0).getSecond();

            assertEquals(GLOBALS.getMaxVelocityMetersPerSec(), resolved.maxVelocityMetersPerSec(), 1e-9);
            assertEquals(0.0, resolved.minVelocityMetersPerSec(), 1e-9);
            assertTrue(sawWarning.get(), "Expected a warning for min/max conflict");
        } finally {
            logger.removeHandler(handler);
        }
    }
}
