package frc.robot.lib.BLine.path;

import java.util.List;
import java.util.Optional;
import java.util.Arrays;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import static org.junit.jupiter.api.Assertions.*;

class PathTest {
    private static final Path.DefaultGlobalConstraints GLOBALS =
        new Path.DefaultGlobalConstraints(5.0, 6.0, 700.0, 1400.0, 0.05, 2.0, 0.2);

    private static Path path(Path.PathElement... elements) {
        return new Path(List.of(elements), null, GLOBALS);
    }

    @Test
    void validityAndExecutionAgreeAfterEditingConstraintsOrElements() {
        Path p = path(new Path.TranslationTarget(1, 2));
        p.setPathConstraints(new Path.PathConstraints().setMaxAccelerationMetersPerSec2(0));
        assertFalse(p.isValid());
        assertThrows(IllegalArgumentException.class, () -> PreparedPath.create(p, Optional.empty(), Optional.empty()));
        assertEquals(List.of(new Translation2d(1, 2)), p.getTranslations(), "Geometry does not require valid motion constraints");
        p.setPathConstraints(new Path.PathConstraints().setMaxAccelerationMetersPerSec2(2));
        assertTrue(p.isValid());
        assertDoesNotThrow(() -> PreparedPath.create(p, Optional.empty(), Optional.empty()));
        p.setElement(0, new Path.RotationTarget(new Rotation2d(), .5));
        assertFalse(p.isValid());
    }

    @Test
    void reorderingRejectsLostOrDuplicatedElementsWithoutChangingThePath() {
        Path p = path(new Path.TranslationTarget(1, 0), new Path.TranslationTarget(2, 0), new Path.TranslationTarget(3, 0));
        var original = p.getPathElements();
        for (List<Integer> order : Arrays.asList(null, List.of(0, 1), List.of(0, 0, 2),
            List.of(-1, 1, 2), List.of(0, 1, 3), Arrays.asList(0, null, 2))) {
            assertThrows(IllegalArgumentException.class, () -> p.reorderElements(order));
            assertEquals(original, p.getPathElements());
        }
        p.reorderElements(List.of(2, 0, 1));
        assertEquals(List.of(original.get(2), original.get(0), original.get(1)), p.getPathElements());
    }

    @Test
    void authoredStartNeverUsesTheDestinationAndMatchesExecutionResetHeading() {
        for (Path p : List.of(path(new Path.Waypoint(4, 3, Rotation2d.fromDegrees(90))),
            path(new Path.RotationTarget(new Rotation2d(), .5), new Path.TranslationTarget(4, 3)),
            path(new Path.EventTrigger(.2, "intake"), new Path.TranslationTarget(4, 3)))) {
            assertTrue(p.getAuthoredStartPose().isEmpty());
            assertThrows(IllegalStateException.class, p::getStartPose);
        }
        Path p = path(new Path.TranslationTarget(1, 2), new Path.Waypoint(4, 3, Rotation2d.fromDegrees(90)));
        var fallback = Rotation2d.fromDegrees(30);
        assertEquals(new Pose2d(1, 2, fallback), p.getStartPose(fallback));
        assertEquals(PreparedPath.create(p, Optional.empty(), Optional.empty()).authoredStart().orElseThrow().pose(fallback),
            p.getAuthoredStartPose(fallback).orElseThrow());
        p.setElement(0, new Path.Waypoint(1, 2, Rotation2d.fromDegrees(15)));
        p.setFlipped(true).setMirrored(true);
        var first = (Path.Waypoint) p.getPathElements().getFirst();
        assertEquals(new Pose2d(first.translationTarget().translation(), first.rotationTarget().rotation()), p.getStartPose(fallback));
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
