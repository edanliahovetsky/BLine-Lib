package frc.robot.lib.BLine.path;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PathTest {
    private static final Path.DefaultGlobalConstraints GLOBALS =
        new Path.DefaultGlobalConstraints(5.0, 6.0, 700.0, 1400.0, 0.05, 2.0, 0.2);

    private static Path path(Path.PathElement... elements) {
        return new Path(List.of(elements), null, GLOBALS);
    }

    @Test
    void getTranslationsReturnsTranslationVerticesExpandingWaypoints() {
        Path p = path(
            new Path.Waypoint(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(45.0))),
            new Path.RotationTarget(Rotation2d.fromDegrees(90.0), 0.5),
            new Path.TranslationTarget(new Translation2d(1.0, 0.0)),
            new Path.Waypoint(new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(180.0)))
        );

        assertEquals(
            List.of(
                new Translation2d(0.0, 0.0),
                new Translation2d(1.0, 0.0),
                new Translation2d(2.0, 0.0)
            ),
            p.getTranslations()
        );
    }

    @Test
    void getTranslationsHandlesSinglePointAndInvalidPath() {
        Path singlePoint = path(new Path.TranslationTarget(2.0, 3.0));

        assertEquals(
            List.of(new Translation2d(2.0, 3.0)),
            singlePoint.getTranslations()
        );

        Path invalid = path(new Path.RotationTarget(Rotation2d.fromDegrees(90.0), 0.5));
        assertTrue(!invalid.isValid());
        assertTrue(invalid.getTranslations().isEmpty());
    }
}
