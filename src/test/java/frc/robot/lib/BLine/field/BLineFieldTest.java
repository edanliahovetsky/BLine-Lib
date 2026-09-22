package frc.robot.lib.BLine.field;

import frc.robot.lib.BLine.path.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.smartdashboard.Field2d;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BLineFieldTest {
    private static final Path.DefaultGlobalConstraints GLOBALS =
        new Path.DefaultGlobalConstraints(5.0, 6.0, 700.0, 1400.0, 0.05, 2.0, 0.2);

    private static final double EPS = 1e-9;

    private static Path path(Path.PathElement... elements) {
        return new Path(List.of(elements), null, GLOBALS);
    }

    @Test
    void drawPathRotatesDisplayPosesTowardTheNextVertex() {
        Field2d field = new Field2d();
        Path p = path(
            new Path.TranslationTarget(0.0, 0.0),
            new Path.TranslationTarget(1.0, 0.0),
            new Path.TranslationTarget(1.0, 1.0)
        );

        String objectName = BLineField.drawPath(field, "MyPath", p);
        List<Pose2d> poses = field.getObject(objectName).getPoses();

        assertEquals("MyPathTrajectory", objectName);
        assertEquals(3, poses.size());
        assertEquals(new Translation2d(0.0, 0.0), poses.get(0).getTranslation());
        assertEquals(0.0, poses.get(0).getRotation().getDegrees(), EPS);
        assertEquals(new Translation2d(1.0, 0.0), poses.get(1).getTranslation());
        assertEquals(90.0, poses.get(1).getRotation().getDegrees(), EPS);
        assertEquals(new Translation2d(1.0, 1.0), poses.get(2).getTranslation());
        assertEquals(90.0, poses.get(2).getRotation().getDegrees(), EPS);
    }

    @Test
    void drawPathSkipsCoincidentPointsWhenChoosingDisplayDirection() {
        Field2d field = new Field2d();
        Path p = path(
            new Path.TranslationTarget(0.0, 0.0),
            new Path.TranslationTarget(0.0, 0.0),
            new Path.TranslationTarget(0.0, 2.0)
        );

        String objectName = BLineField.drawPath(field, "MyPath", p);
        List<Pose2d> poses = field.getObject(objectName).getPoses();

        assertEquals(3, poses.size());
        assertEquals(90.0, poses.get(0).getRotation().getDegrees(), EPS);
        assertEquals(90.0, poses.get(1).getRotation().getDegrees(), EPS);
        assertEquals(90.0, poses.get(2).getRotation().getDegrees(), EPS);
    }

    @Test
    void automaticallyNamedPathsAreUniqueForAField() {
        Field2d field = new Field2d();
        Path firstPath = path(
            new Path.TranslationTarget(0.0, 0.0),
            new Path.TranslationTarget(1.0, 0.0)
        );
        Path secondPath = path(
            new Path.TranslationTarget(2.0, 0.0),
            new Path.TranslationTarget(3.0, 0.0)
        );

        String firstName = BLineField.drawPath(field, firstPath);
        String secondName = BLineField.drawPath(field, secondPath);

        assertNotEquals(firstName, secondName);
        assertEquals("BLinePath0Trajectory", firstName);
        assertEquals("BLinePath1Trajectory", secondName);
        assertEquals(firstPath.getTranslations(), field.getObject(firstName).getPoses().stream().map(Pose2d::getTranslation).toList());
        assertEquals(secondPath.getTranslations(), field.getObject(secondName).getPoses().stream().map(Pose2d::getTranslation).toList());
    }

    @Test
    void automaticallyNamedPathReusesNameForSamePathInstance() {
        Field2d field = new Field2d();
        Path p = path(
            new Path.TranslationTarget(0.0, 0.0),
            new Path.TranslationTarget(1.0, 0.0)
        );

        String firstName = BLineField.drawPath(field, p);
        p.setElement(1, new Path.TranslationTarget(2.0, 0.0));
        String secondName = BLineField.drawPath(field, p);

        assertEquals(firstName, secondName);
        assertEquals(List.of(new Translation2d(0.0, 0.0), new Translation2d(2.0, 0.0)),
            field.getObject(firstName).getPoses().stream().map(Pose2d::getTranslation).toList());
    }

    @Test
    void explicitNamesAppendTrajectorySuffixOnce() {
        Field2d field = new Field2d();
        Path p = path(new Path.TranslationTarget(0.0, 0.0));

        assertEquals("SharedTrajectory", BLineField.drawPath(field, "Shared", p));
        assertEquals("SharedTrajectory", BLineField.drawPath(field, "SharedTrajectory", p));
        assertEquals("sharedtrajectory", BLineField.drawPath(field, "sharedtrajectory", p));
    }

    @Test
    void explicitNameReuseUpdatesTheSameDisplaySlot() {
        Field2d field = new Field2d();
        Path firstPath = path(
            new Path.TranslationTarget(0.0, 0.0),
            new Path.TranslationTarget(1.0, 0.0)
        );
        Path secondPath = path(
            new Path.TranslationTarget(2.0, 0.0),
            new Path.TranslationTarget(3.0, 0.0)
        );

        String firstName = BLineField.drawPath(field, "Shared", firstPath);
        String secondName = BLineField.drawPath(field, "Shared", secondPath);

        assertEquals(firstName, secondName);
        assertEquals(secondPath.getTranslations(), field.getObject(firstName).getPoses().stream().map(Pose2d::getTranslation).toList());
    }

    @Test
    void rejectsInvalidInputs() {
        Field2d field = new Field2d();
        Path p = path(new Path.TranslationTarget(0.0, 0.0));

        assertThrows(NullPointerException.class, () -> BLineField.drawPath(null, p));
        assertThrows(NullPointerException.class, () -> BLineField.drawPath(field, (Path) null));
        assertThrows(NullPointerException.class, () -> BLineField.drawPath(null, "Path", p));
        assertThrows(NullPointerException.class, () -> BLineField.drawPath(field, null, p));
        assertThrows(NullPointerException.class, () -> BLineField.drawPath(field, "Path", null));
        assertThrows(IllegalArgumentException.class, () -> BLineField.drawPath(field, " ", p));
    }

    @Test
    void autoGeneratedNamesAlwaysEndWithTrajectory() {
        Field2d field = new Field2d();
        Path p = path(new Path.TranslationTarget(0.0, 0.0));

        assertTrue(BLineField.drawPath(field, p).endsWith("Trajectory"));
    }
}
