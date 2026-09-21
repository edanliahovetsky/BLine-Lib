package frc.robot.lib.BLine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.util.Pair;
import org.wpilib.simulation.DriverStationSim;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.PathElementConstraint;
import frc.robot.lib.BLine.Path.TranslationTargetConstraint;
import java.io.File;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class BehavioralCompatibilityTest {
    private static final double EPSILON = 1e-9;
    private static final List<Translation2d> EXPECTED_TRANSLATIONS = List.of(
        new Translation2d(1.25, 2.50),
        new Translation2d(4.75, 3.25),
        new Translation2d(7.00, 1.50)
    );

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private double originalFieldSizeX;
    private double originalFieldSizeY;
    private FlippingUtil.FieldSymmetry originalSymmetryType;

    @BeforeEach
    void setUp() {
        originalFieldSizeX = FlippingUtil.fieldSizeX;
        originalFieldSizeY = FlippingUtil.fieldSizeY;
        originalSymmetryType = FlippingUtil.symmetryType;
        resetDriverStation();
    }

    @AfterEach
    void tearDown() {
        resetScheduler();
        scheduler.enable();
        resetDriverStation();
        FlippingUtil.fieldSizeX = originalFieldSizeX;
        FlippingUtil.fieldSizeY = originalFieldSizeY;
        FlippingUtil.symmetryType = originalSymmetryType;
    }

    private static void resetDriverStation() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test
    void representativePathLoadsGeometryConstraintsAndDefaultsFromRealFiles() throws URISyntaxException {
        Path path = loadRepresentativePath();

        assertTrue(path.isValid());
        assertEquals(EXPECTED_TRANSLATIONS, path.getTranslations());
        assertRepresentativeElements(path.getPathElements());
        assertEquals(0.04, path.getEndTranslationToleranceMeters(), EPSILON);
        assertEquals(1.75, path.getEndRotationToleranceDeg(), EPSILON);

        Path.DefaultGlobalConstraints defaults = path.getDefaultGlobalConstraints();
        assertEquals(4.80, defaults.getMaxVelocityMetersPerSec(), EPSILON);
        assertEquals(6.40, defaults.getMaxAccelerationMetersPerSec2(), EPSILON);
        assertEquals(690.0, defaults.getMaxVelocityDegPerSec(), EPSILON);
        assertEquals(1320.0, defaults.getMaxAccelerationDegPerSec2(), EPSILON);
        assertEquals(0.06, defaults.getEndTranslationToleranceMeters(), EPSILON);
        assertEquals(2.25, defaults.getEndRotationToleranceDeg(), EPSILON);
        assertEquals(0.28, defaults.getIntermediateHandoffRadiusMeters(), EPSILON);

        Path.PathConstraints constraints = path.getPathConstraints();
        assertRangedConstraint(constraints.getMaxVelocityMetersPerSec().orElseThrow().get(0), 2.40, 0, 0);
        assertRangedConstraint(constraints.getMaxVelocityMetersPerSec().orElseThrow().get(1), 3.10, 1, 2);
        assertRangedConstraint(constraints.getMaxAccelerationMetersPerSec2().orElseThrow().get(0), 3.60, 0, 2);
        assertRangedConstraint(constraints.getMaxVelocityDegPerSec().orElseThrow().get(0), 420.0, 0, 2);
        assertRangedConstraint(constraints.getMaxAccelerationDegPerSec2().orElseThrow().get(0), 880.0, 0, 2);
        assertRangedConstraint(constraints.getMinVelocityMetersPerSec().orElseThrow().get(0), 0.55, 0, 0);
        assertRangedConstraint(constraints.getMinVelocityDegPerSec().orElseThrow().get(0), 35.0, 0, 2);

        List<Pair<PathElement, PathElementConstraint>> resolved = path.getPathElementsWithConstraintsNoWaypoints();
        List<TranslationTargetConstraint> translationConstraints = resolved.stream()
            .filter(pair -> pair.getFirst() instanceof Path.TranslationTarget)
            .map(pair -> (TranslationTargetConstraint) pair.getSecond())
            .toList();
        TranslationTargetConstraint firstTranslation = translationConstraints.get(0);
        TranslationTargetConstraint secondTranslation = translationConstraints.get(1);
        assertEquals(2.40, firstTranslation.maxVelocityMetersPerSec(), EPSILON);
        assertEquals(3.60, firstTranslation.maxAccelerationMetersPerSec2(), EPSILON);
        assertEquals(0.55, firstTranslation.minVelocityMetersPerSec(), EPSILON);
        assertEquals(3.10, secondTranslation.maxVelocityMetersPerSec(), EPSILON);
    }

    @Test
    void existingFieldDefaultsRemainStable() {
        assertEquals(16.54, FlippingUtil.fieldSizeX, EPSILON);
        assertEquals(8.07, FlippingUtil.fieldSizeY, EPSILON);
        assertEquals(FlippingUtil.FieldSymmetry.kRotational, FlippingUtil.symmetryType);
    }

    @Test
    void publicPathAndBuilderUseConfiguredFieldDimensionsForFlipAndMirror() throws URISyntaxException {
        FlippingUtil.fieldSizeX = 18.0;
        FlippingUtil.fieldSizeY = 9.0;

        Path flippedPath = loadRepresentativePath();
        flippedPath.flip();
        assertEquals(
            List.of(
                new Translation2d(16.75, 6.50),
                new Translation2d(13.25, 5.75),
                new Translation2d(11.00, 7.50)
            ),
            flippedPath.getTranslations()
        );
        assertRepresentativeRotations(flippedPath, -150.0, -90.0, 135.0);

        Path mirroredPath = loadRepresentativePath();
        mirroredPath.mirror();
        assertEquals(
            List.of(
                new Translation2d(1.25, 6.50),
                new Translation2d(4.75, 5.75),
                new Translation2d(7.00, 7.50)
            ),
            mirroredPath.getTranslations()
        );
        assertRepresentativeRotations(mirroredPath, -30.0, -90.0, 45.0);

        MutableRobot flippedRobot = new MutableRobot();
        Command flippedCommand = builder(flippedRobot)
            .withShouldFlip(() -> true)
            .withPoseReset(flippedRobot::setPose)
            .build(loadRepresentativePath())
            .ignoringDisable(true);

        scheduleAndRunOnce(flippedCommand);
        assertPose(flippedRobot.pose, new Pose2d(16.75, 6.50, Rotation2d.fromDegrees(-150.0)));

        resetScheduler();

        MutableRobot mirroredRobot = new MutableRobot();
        Command mirroredCommand = builder(mirroredRobot)
            .withShouldMirror(() -> true)
            .withPoseReset(mirroredRobot::setPose)
            .build(loadRepresentativePath())
            .ignoringDisable(true);

        scheduleAndRunOnce(mirroredCommand);
        assertPose(mirroredRobot.pose, new Pose2d(1.25, 6.50, Rotation2d.fromDegrees(-30.0)));
    }

    @Test
    void defaultAllianceSupplierFlipsFollowerForRedAlliance() throws URISyntaxException {
        DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
        DriverStationSim.notifyNewData();
        MutableRobot robot = new MutableRobot();
        Command command = builder(robot)
            .withDefaultShouldFlip()
            .withPoseReset(robot::setPose)
            .build(loadRepresentativePath())
            .ignoringDisable(true);

        scheduleAndRunOnce(command);

        assertPose(robot.pose, new Pose2d(15.29, 5.57, Rotation2d.fromDegrees(-150.0)));
    }

    @Test
    void followerObservesMutableRobotPoseThroughRealCommandScheduler() throws URISyntaxException {
        MutableRobot robot = new MutableRobot();
        FollowPath follower = builder(robot)
            .withPoseReset(robot::setPose)
            .build(loadRepresentativePath());
        Command scheduledFollower = follower.ignoringDisable(true);
        AtomicBoolean markerRan = new AtomicBoolean(false);
        FollowPath.registerEventTrigger(
            "behavior-parity-marker",
            Commands.runOnce(() -> markerRan.set(true)).ignoringDisable(true)
        );

        scheduler.schedule(scheduledFollower);
        scheduler.run();

        assertTrue(scheduler.isScheduled(scheduledFollower));
        assertPose(robot.pose, new Pose2d(1.25, 2.50, Rotation2d.fromDegrees(30.0)));
        assertTrue(
            Math.hypot(robot.commandedSpeeds.vx, robot.commandedSpeeds.vy) > 0.0,
            "Follower should command translation while the mutable pose is at the path start"
        );

        robot.setPose(new Pose2d(4.75, 3.25, Rotation2d.fromDegrees(90.0)));
        for (int cycle = 0; cycle < 3; cycle++) {
            scheduler.run();
        }
        assertTrue(markerRan.get(), "Loaded event marker should schedule a real Commands v2 command");

        robot.setPose(new Pose2d(7.00, 1.50, Rotation2d.fromDegrees(-45.0)));
        for (int cycle = 0; cycle < 12 && scheduler.isScheduled(scheduledFollower); cycle++) {
            scheduler.run();
        }

        assertFalse(scheduler.isScheduled(scheduledFollower), "Follower should finish at the loaded path endpoint");
        assertEquals(0.0, robot.commandedSpeeds.vx, EPSILON);
        assertEquals(0.0, robot.commandedSpeeds.vy, EPSILON);
        assertEquals(0.0, robot.commandedSpeeds.omega, EPSILON);
    }

    private static FollowPath.Builder builder(MutableRobot robot) {
        return new FollowPath.Builder(
            new TestDriveSubsystem(),
            robot::getPose,
            robot::getSpeeds,
            robot::acceptSpeeds,
            new PIDController(2.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.0),
            new PIDController(1.0, 0.0, 0.0)
        );
    }

    private void scheduleAndRunOnce(Command command) {
        scheduler.schedule(command);
        scheduler.run();
        assertTrue(scheduler.isScheduled(command));
    }

    private void resetScheduler() {
        scheduler.cancelAll();
        scheduler.clearComposedCommands();
    }

    private static Path loadRepresentativePath() throws URISyntaxException {
        return new Path(resourceAutosDirectory(), "behavior-parity");
    }

    private static void assertRepresentativeElements(List<PathElement> elements) {
        assertEquals(5, elements.size());
        Path.Waypoint start = (Path.Waypoint) elements.get(0);
        assertEquals(0.30, start.translationTarget().intermediateHandoffRadiusMeters().orElseThrow(), EPSILON);
        assertEquals(30.0, start.rotationTarget().rotation().getDegrees(), EPSILON);
        assertEquals(0.0, start.rotationTarget().t_ratio(), EPSILON);
        assertFalse(start.rotationTarget().profiledRotation());

        Path.EventTrigger event = (Path.EventTrigger) elements.get(1);
        assertEquals(0.45, event.t_ratio(), EPSILON);
        assertEquals("behavior-parity-marker", event.libKey());

        Path.RotationTarget rotation = (Path.RotationTarget) elements.get(2);
        assertEquals(90.0, rotation.rotation().getDegrees(), EPSILON);
        assertEquals(0.55, rotation.t_ratio(), EPSILON);
        assertTrue(rotation.profiledRotation());

        Path.TranslationTarget translation = (Path.TranslationTarget) elements.get(3);
        assertEquals(0.22, translation.intermediateHandoffRadiusMeters().orElseThrow(), EPSILON);

        Path.Waypoint end = (Path.Waypoint) elements.get(4);
        assertTrue(end.translationTarget().intermediateHandoffRadiusMeters().isEmpty());
        assertEquals(-45.0, end.rotationTarget().rotation().getDegrees(), EPSILON);
        assertEquals(1.0, end.rotationTarget().t_ratio(), EPSILON);
        assertFalse(end.rotationTarget().profiledRotation());
    }

    private static void assertRepresentativeRotations(
        Path path,
        double startDegrees,
        double intermediateDegrees,
        double endDegrees
    ) {
        List<PathElement> elements = path.getPathElements();
        assertEquals(startDegrees, ((Path.Waypoint) elements.get(0)).rotationTarget().rotation().getDegrees(), EPSILON);
        assertEquals(intermediateDegrees, ((Path.RotationTarget) elements.get(2)).rotation().getDegrees(), EPSILON);
        assertEquals(endDegrees, ((Path.Waypoint) elements.get(4)).rotationTarget().rotation().getDegrees(), EPSILON);
    }

    private static void assertRangedConstraint(
        Path.RangedConstraint constraint,
        double value,
        int startOrdinal,
        int endOrdinal
    ) {
        assertEquals(value, constraint.value(), EPSILON);
        assertEquals(startOrdinal, constraint.startOrdinal());
        assertEquals(endOrdinal, constraint.endOrdinal());
    }

    private static File resourceAutosDirectory() throws URISyntaxException {
        URL resource = BehavioralCompatibilityTest.class.getResource("/behavior-parity/autos");
        assertNotNull(resource, "Representative autos test resource must be present");
        return new File(resource.toURI());
    }

    private static void assertPose(Pose2d actual, Pose2d expected) {
        assertEquals(expected.getX(), actual.getX(), EPSILON);
        assertEquals(expected.getY(), actual.getY(), EPSILON);
        assertEquals(expected.getRotation().getDegrees(), actual.getRotation().getDegrees(), EPSILON);
    }

    private static final class TestDriveSubsystem implements Subsystem {}

    private static final class MutableRobot {
        private Pose2d pose = new Pose2d();
        private ChassisVelocities commandedSpeeds = new ChassisVelocities();

        private Pose2d getPose() {
            return pose;
        }

        private void setPose(Pose2d pose) {
            this.pose = pose;
        }

        private ChassisVelocities getSpeeds() {
            return commandedSpeeds;
        }

        private void acceptSpeeds(ChassisVelocities speeds) {
            commandedSpeeds = speeds;
        }
    }
}
