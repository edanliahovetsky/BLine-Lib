package frc.robot.lib.BLine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FollowPathTest {
    private static final AtomicInteger EVENT_KEY_COUNTER = new AtomicInteger(0);
    private static final Path.DefaultGlobalConstraints TEST_GLOBAL_CONSTRAINTS =
        new Path.DefaultGlobalConstraints(
            4.0,
            4.0,
            720.0,
            1440.0,
            0.05,
            2.0,
            0.20
        );

    @BeforeEach
    void setUp() {
        Path.setDefaultGlobalConstraints(TEST_GLOBAL_CONSTRAINTS);
        FollowPath.setPoseLoggingConsumer(value -> {});
        FollowPath.setDoubleLoggingConsumer(value -> {});
        FollowPath.setBooleanLoggingConsumer(value -> {});
        FollowPath.setTranslationListLoggingConsumer(value -> {});
        FollowPath.clearRotationOverride();
    }

    @AfterEach
    void tearDown() {
        FollowPath.setTimestampSupplier(null);
        FollowPath.setPoseLoggingConsumer(value -> {});
        FollowPath.setDoubleLoggingConsumer(value -> {});
        FollowPath.setBooleanLoggingConsumer(value -> {});
        FollowPath.setTranslationListLoggingConsumer(value -> {});
        FollowPath.clearRotationOverride();
    }

    @Test
    void rotationLooksAheadToNextSegmentWhenCurrentSegmentHasNoRotationTargets() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.TranslationTarget(new Translation2d(1.0, 0.0)),
            new Path.RotationTarget(Rotation2d.fromDegrees(90.0), 0.5),
            new Path.TranslationTarget(new Translation2d(2.0, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        assertEquals(1, command.getCurrentTranslationElementIndex(), "Expected handoff to second translation target");
        assertEquals(2, command.getCurrentRotationElementIndex(), "Expected lookahead to the next available rotation target");
    }

    @Test
    void rotationLooksAheadAcrossMultipleSegmentsWhenNeeded() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.TranslationTarget(new Translation2d(1.0, 0.0)),
            new Path.TranslationTarget(new Translation2d(2.0, 0.0)),
            new Path.RotationTarget(Rotation2d.fromDegrees(180.0), 0.5),
            new Path.TranslationTarget(new Translation2d(3.0, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        assertEquals(1, command.getCurrentTranslationElementIndex(), "Expected handoff to current segment endpoint");
        assertEquals(3, command.getCurrentRotationElementIndex(), "Expected lookahead to later-segment rotation target");
    }

    @Test
    void zeroLengthSegmentUsesHighestTRatioRotationAndCanFinish() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.RotationTarget(Rotation2d.fromDegrees(30.0), 0.2),
            new Path.RotationTarget(Rotation2d.fromDegrees(120.0), 0.9),
            new Path.TranslationTarget(new Translation2d(0.0, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        assertEquals(3, command.getCurrentTranslationElementIndex(), "Expected immediate handoff through zero-length segment");
        assertEquals(2, command.getCurrentRotationElementIndex(), "Expected highest t_ratio rotation target to remain active");
        assertFalse(command.isFinished(), "Should not finish until final rotation is reached");

        robot.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(120.0)));
        runExecute(command, robot);

        assertTrue(command.isFinished(), "Should finish once final degenerate-segment rotation is achieved");
    }

    @Test
    void multiWaypointPathStillFinishesNormally() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.Waypoint(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0))),
            new Path.Waypoint(new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(45.0))),
            new Path.Waypoint(new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(90.0)))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();

        runExecute(command, robot);
        robot.setPose(new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(45.0)));
        runExecute(command, robot);
        robot.setPose(new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(90.0)));
        runExecute(command, robot);
        runExecute(command, robot);

        assertTrue(command.isFinished(), "Expected multi-waypoint path to complete with end tolerances");
    }

    @Test
    void singleTranslationTargetPathCanFinish() {
        MutableRobot robot = new MutableRobot(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(15.0)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.TranslationTarget(new Translation2d(1.0, 2.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        assertEquals(0, command.getCurrentTranslationElementIndex(), "Single translation target path should remain on only translation target");
        assertTrue(command.isFinished(), "Single translation target path should finish when already at setpoint");
    }

    @Test
    void singleWaypointPathCanFinish() {
        MutableRobot robot = new MutableRobot(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.Waypoint(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0)))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        assertEquals(0, command.getCurrentTranslationElementIndex(), "Single waypoint path should remain on only translation target");
        assertTrue(command.isFinished(), "Single waypoint path should finish when translation and rotation are at setpoint");
    }

    @Test
    void tratioHandoffStillAdvancesWhenRobotStartsInsideCurrentTargetRadius() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.TranslationTarget(new Translation2d(0.0005, 0.0)),
            new Path.TranslationTarget(new Translation2d(1.0, 0.0))
        );

        FollowPath command = createCommand(path, robot, true);
        command.initialize();
        runExecute(command, robot);

        assertEquals(
            1,
            command.getCurrentTranslationElementIndex(),
            "t-ratio mode should still hand off when the robot already starts within the current target radius"
        );
    }

    @Test
    void remainingPathDistanceIsZeroBeforeInitialization() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        Path path = new Path(
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.TranslationTarget(new Translation2d(1.0, 0.0))
        );

        FollowPath command = createCommand(path, robot);

        assertEquals(
            0.0,
            command.getRemainingPathDistanceMeters(),
            1e-9,
            "Distance getter should return 0.0 before command initialization"
        );
    }

    @Test
    void remainingPathDistanceDecreasesAsRobotProgresses() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.TranslationTarget(new Translation2d(1.0, 0.0)),
            new Path.TranslationTarget(new Translation2d(2.0, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();

        runExecute(command, robot);
        double firstDistance = command.getRemainingPathDistanceMeters();

        robot.setPose(new Pose2d(1.0, 0.0, new Rotation2d()));
        runExecute(command, robot);
        double secondDistance = command.getRemainingPathDistanceMeters();

        robot.setPose(new Pose2d(2.0, 0.0, new Rotation2d()));
        runExecute(command, robot);
        double finalDistance = command.getRemainingPathDistanceMeters();

        assertTrue(firstDistance > secondDistance, "Remaining distance should shrink as robot advances");
        assertTrue(secondDistance > finalDistance, "Remaining distance should continue shrinking near path end");
        assertEquals(0.0, finalDistance, 1e-6, "Remaining distance should be ~0 at final target");
    }

    @Test
    void remainingPathDistanceIsZeroForInvalidPath() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        Path invalidPath = new Path(
            new Path.RotationTarget(Rotation2d.fromDegrees(45.0), 0.5)
        );

        FollowPath command = createCommand(invalidPath, robot);
        command.initialize();

        assertEquals(
            0.0,
            command.getRemainingPathDistanceMeters(),
            1e-9,
            "Distance getter should return 0.0 when path is invalid"
        );
    }

    @Test
    void rotationTargetPoseLoggingUsesTargetRotationAndNewKey() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Map<String, Pose2d> poseLogs = new HashMap<>();
        FollowPath.setPoseLoggingConsumer((Pair<String, Pose2d> pair) -> poseLogs.put(pair.getFirst(), pair.getSecond()));

        Path path = new Path(
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.RotationTarget(Rotation2d.fromDegrees(90.0), 0.5),
            new Path.TranslationTarget(new Translation2d(2.0, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        Pose2d rotationTargetPoseLog = poseLogs.get("FollowPath/rotationTargetPose");
        assertNotNull(rotationTargetPoseLog, "Expected FollowPath/rotationTargetPose log to be emitted");
        assertEquals(
            90.0,
            rotationTargetPoseLog.getRotation().getDegrees(),
            1e-9,
            "rotationTargetPose log should carry the rotation target's heading"
        );
        assertFalse(
            poseLogs.containsKey("FollowPath/calculateRotationTargetTranslation"),
            "Old rotation target translation log key should no longer be emitted"
        );
    }

    @Test
    void radiusBasedTranslationHandoffRequiresDistanceToTarget() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.TranslationTarget(0.0, 0.0, 0.20),
            new Path.TranslationTarget(10.0, 0.0, 0.20),
            new Path.TranslationTarget(20.0, 0.0, 0.20)
        );

        FollowPath command = createCommand(path, robot, false);
        command.initialize();

        runExecute(command, robot);
        assertEquals(1, command.getCurrentTranslationElementIndex(), "Expected to hand off from first target to second");

        // Close in projected t-ratio, but physically far from the target due to lateral offset.
        robot.setPose(new Pose2d(9.9, 5.0, new Rotation2d()));
        runExecute(command, robot);

        assertEquals(
            1,
            command.getCurrentTranslationElementIndex(),
            "Radius-based handoff should not advance when outside handoff radius"
        );
    }

    @Test
    void tRatioBasedTranslationHandoffUsesSegmentProgress() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.TranslationTarget(0.0, 0.0, 0.20),
            new Path.TranslationTarget(10.0, 0.0, 0.20),
            new Path.TranslationTarget(20.0, 0.0, 0.20)
        );

        FollowPath command = createCommand(path, robot, true);
        command.initialize();

        runExecute(command, robot);
        assertEquals(1, command.getCurrentTranslationElementIndex(), "Expected to hand off from first target to second");

        // Same pose as radius test: high projected progress but outside physical radius.
        robot.setPose(new Pose2d(9.9, 5.0, new Rotation2d()));
        runExecute(command, robot);

        assertEquals(
            2,
            command.getCurrentTranslationElementIndex(),
            "t-ratio handoff should advance based on projected segment progress"
        );
    }

    @Test
    void translationAndRotationMustBothBeSatisfiedToFinish() {
        MutableRobot robot = new MutableRobot(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(0.0)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.Waypoint(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0)))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        assertFalse(command.isFinished(), "Should not finish when rotation is outside end tolerance");

        robot.setPose(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0)));
        runExecute(command, robot);
        assertTrue(command.isFinished(), "Should finish once both translation and rotation are at setpoint");
    }

    @Test
    void notFinishedWhenTranslationErrorRemainsEvenIfRotationMatches() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.Waypoint(new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(0.0)))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        assertFalse(command.isFinished(), "Should not finish while translation error remains above tolerance");
    }

    @Test
    void translationMinimumBaselineRaisesControllerOutputOutsideTolerance() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Map<String, Double> doubleLogs = new HashMap<>();
        Map<String, Boolean> booleanLogs = new HashMap<>();
        FollowPath.setDoubleLoggingConsumer((Pair<String, Double> pair) -> doubleLogs.put(pair.getFirst(), pair.getSecond()));
        FollowPath.setBooleanLoggingConsumer((Pair<String, Boolean> pair) -> booleanLogs.put(pair.getFirst(), pair.getSecond()));

        Path path = new Path(
            new Path.PathConstraints()
                .setMaxVelocityMetersPerSec(2.0)
                .setMinVelocityMetersPerSec(1.25)
                .setMaxAccelerationMetersPerSec2(1000.0)
                .setEndTranslationToleranceMeters(0.01),
            new Path.TranslationTarget(new Translation2d(0.10, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        assertTrue(doubleLogs.get("FollowPath/rawTranslationControllerOutput") < 1.25);
        assertEquals(1.25, doubleLogs.get("FollowPath/translationControllerOutput"), 1e-9);
        assertTrue(booleanLogs.get("FollowPath/translationMinimumApplied"));
    }

    @Test
    void translationMinimumBaselineTurnsOffInsideTolerance() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Map<String, Double> doubleLogs = new HashMap<>();
        Map<String, Boolean> booleanLogs = new HashMap<>();
        FollowPath.setDoubleLoggingConsumer((Pair<String, Double> pair) -> doubleLogs.put(pair.getFirst(), pair.getSecond()));
        FollowPath.setBooleanLoggingConsumer((Pair<String, Boolean> pair) -> booleanLogs.put(pair.getFirst(), pair.getSecond()));

        Path path = new Path(
            new Path.PathConstraints()
                .setMaxVelocityMetersPerSec(2.0)
                .setMinVelocityMetersPerSec(1.25),
            new Path.TranslationTarget(new Translation2d(0.02, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        assertEquals(
            doubleLogs.get("FollowPath/clampedTranslationControllerOutput"),
            doubleLogs.get("FollowPath/translationControllerOutput"),
            1e-9
        );
        assertFalse(booleanLogs.get("FollowPath/translationMinimumApplied"));
    }

    @Test
    void rotationMinimumBaselineRaisesControllerOutputOutsideTolerance() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Map<String, Double> doubleLogs = new HashMap<>();
        Map<String, Boolean> booleanLogs = new HashMap<>();
        FollowPath.setDoubleLoggingConsumer((Pair<String, Double> pair) -> doubleLogs.put(pair.getFirst(), pair.getSecond()));
        FollowPath.setBooleanLoggingConsumer((Pair<String, Boolean> pair) -> booleanLogs.put(pair.getFirst(), pair.getSecond()));

        Path path = new Path(
            new Path.PathConstraints()
                .setMaxVelocityDegPerSec(180.0)
                .setMinVelocityDegPerSec(60.0)
                .setMaxAccelerationDegPerSec2(10000.0)
                .setEndRotationToleranceDeg(0.5),
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.RotationTarget(Rotation2d.fromDegrees(5.0), 0.5, false),
            new Path.TranslationTarget(new Translation2d(1.0, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        assertTrue(doubleLogs.get("FollowPath/rawRotationControllerOutput") < Math.toRadians(60.0));
        assertEquals(Math.toRadians(60.0), doubleLogs.get("FollowPath/rotationControllerOutput"), 1e-9);
        assertTrue(booleanLogs.get("FollowPath/rotationMinimumApplied"));
    }

    @Test
    void rotationMinimumBaselineTurnsOffInsideTolerance() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(4.8)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Map<String, Double> doubleLogs = new HashMap<>();
        Map<String, Boolean> booleanLogs = new HashMap<>();
        FollowPath.setDoubleLoggingConsumer((Pair<String, Double> pair) -> doubleLogs.put(pair.getFirst(), pair.getSecond()));
        FollowPath.setBooleanLoggingConsumer((Pair<String, Boolean> pair) -> booleanLogs.put(pair.getFirst(), pair.getSecond()));

        Path path = new Path(
            new Path.PathConstraints()
                .setMaxVelocityDegPerSec(180.0)
                .setMinVelocityDegPerSec(60.0),
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.RotationTarget(Rotation2d.fromDegrees(5.0), 0.5, false),
            new Path.TranslationTarget(new Translation2d(1.0, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        assertEquals(
            doubleLogs.get("FollowPath/clampedRotationControllerOutput"),
            doubleLogs.get("FollowPath/rotationControllerOutput"),
            1e-9
        );
        assertFalse(booleanLogs.get("FollowPath/rotationMinimumApplied"));
    }

    @Test
    void rotationIndexTransitionsToNoActiveTargetAfterLastRotation() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.RotationTarget(Rotation2d.fromDegrees(90.0), 0.5),
            new Path.TranslationTarget(new Translation2d(1.0, 0.0)),
            new Path.TranslationTarget(new Translation2d(2.0, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        assertEquals(1, command.getCurrentRotationElementIndex(), "Expected active rotation target on current segment");

        robot.setPose(new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(90.0)));
        runExecute(command, robot);

        assertEquals(-1, command.getCurrentRotationElementIndex(), "Expected no active rotation target after last one is completed");
        assertFalse(command.isFinished(), "Should not finish until final translation is reached");

        robot.setPose(new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(90.0)));
        runExecute(command, robot);

        assertTrue(command.isFinished(), "Should finish at final translation while holding final completed rotation");
    }

    @Test
    void eventTriggersFireOnceInOrderAsProgressAdvances() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        String keyA = newEventKey("event-order-a");
        String keyB = newEventKey("event-order-b");
        List<String> firedOrder = new ArrayList<>();
        FollowPath.registerEventTrigger(keyA, () -> firedOrder.add("A"));
        FollowPath.registerEventTrigger(keyB, () -> firedOrder.add("B"));

        Path path = new Path(
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.EventTrigger(0.2, keyA),
            new Path.EventTrigger(0.7, keyB),
            new Path.TranslationTarget(new Translation2d(10.0, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();

        runExecute(command, robot);
        assertEquals(List.of(), firedOrder, "No event should fire at segment progress 0.0");

        robot.setPose(new Pose2d(3.0, 0.0, new Rotation2d()));
        runExecute(command, robot);
        assertEquals(List.of("A"), firedOrder, "First event should fire once when t_ratio is reached");

        robot.setPose(new Pose2d(8.0, 0.0, new Rotation2d()));
        runExecute(command, robot);
        assertEquals(List.of("A", "B"), firedOrder, "Second event should fire after first, in path order");

        robot.setPose(new Pose2d(9.5, 0.0, new Rotation2d()));
        runExecute(command, robot);
        assertEquals(List.of("A", "B"), firedOrder, "Events should not refire once already triggered");
    }

    @Test
    void eventTriggerOnDegenerateSegmentFiresImmediatelyAndOnlyOnce() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        String key = newEventKey("event-degenerate");
        AtomicInteger fireCount = new AtomicInteger(0);
        FollowPath.registerEventTrigger(key, fireCount::incrementAndGet);

        Path path = new Path(
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.EventTrigger(0.5, key),
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.TranslationTarget(new Translation2d(1.0, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();

        runExecute(command, robot);
        assertEquals(1, fireCount.get(), "Degenerate event segment should trigger immediately when processed");

        runExecute(command, robot);
        assertEquals(1, fireCount.get(), "Degenerate event should still fire only once");
    }

    @Test
    void eventTriggerOnFutureSegmentDoesNotFireEarly() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        String key = newEventKey("event-future");
        AtomicInteger fireCount = new AtomicInteger(0);
        FollowPath.registerEventTrigger(key, fireCount::incrementAndGet);

        Path path = new Path(
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.TranslationTarget(new Translation2d(1.0, 0.0)),
            new Path.EventTrigger(0.1, key),
            new Path.TranslationTarget(new Translation2d(2.0, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();

        runExecute(command, robot);
        robot.setPose(new Pose2d(0.6, 0.0, new Rotation2d()));
        runExecute(command, robot);
        assertEquals(0, fireCount.get(), "Future-segment event must not fire while still on current segment");

        robot.setPose(new Pose2d(1.2, 0.0, new Rotation2d()));
        runExecute(command, robot);
        assertEquals(1, fireCount.get(), "Event should fire once robot progresses onto owning segment");
    }

    @Test
    void invalidPathFinishesImmediatelyAndCommandsZeroSpeeds() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        robot.setRobotRelativeSpeeds(new ChassisSpeeds(1.5, -0.3, 0.7));
        Path invalidPath = new Path(
            new Path.RotationTarget(Rotation2d.fromDegrees(30.0), 0.5)
        );

        FollowPath command = createCommand(invalidPath, robot);
        command.initialize();
        runExecute(command, robot);

        assertTrue(command.isFinished(), "Invalid path should finish immediately");
        assertTrue(areSpeedsNearZero(robot.getRobotRelativeSpeeds(), 1e-9), "Invalid path execution should command zero speeds");
    }

    @Test
    void endStopsCommandedMotion() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Path path = new Path(
            new Path.TranslationTarget(new Translation2d(3.0, 0.0))
        );

        FollowPath command = createCommand(path, robot);
        command.initialize();
        runExecute(command, robot);

        assertFalse(areSpeedsNearZero(robot.getRobotRelativeSpeeds(), 1e-9), "Command should produce motion before ending");
        command.end(false);
        assertTrue(areSpeedsNearZero(robot.getRobotRelativeSpeeds(), 1e-9), "end() should command zero speeds");
    }

    @Test
    void rotationOverrideRejectsNullInputs() {
        assertThrows(IllegalArgumentException.class, () -> FollowPath.overrideRotation(null));
        assertThrows(
            IllegalArgumentException.class,
            () -> FollowPath.overrideRotation(() -> 0.0, null)
        );
    }

    @Test
    void normalRotationRunsWhenOverrideInactive() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Map<String, Double> doubleLogs = new HashMap<>();
        Map<String, Boolean> booleanLogs = new HashMap<>();
        FollowPath.setDoubleLoggingConsumer((Pair<String, Double> pair) -> doubleLogs.put(pair.getFirst(), pair.getSecond()));
        FollowPath.setBooleanLoggingConsumer((Pair<String, Boolean> pair) -> booleanLogs.put(pair.getFirst(), pair.getSecond()));

        FollowPath command = createCommand(createDegenerateRotationPath(1.0), robot);
        command.initialize();
        runExecute(command, robot);

        double expectedOmega = 5.0 * Math.toRadians(1.0);
        assertEquals(expectedOmega, robot.getRobotRelativeSpeeds().omegaRadiansPerSecond, 1e-9);
        assertEquals(expectedOmega, doubleLogs.get("FollowPath/rotationPidOutputRadPerSec"), 1e-9);
        assertEquals(expectedOmega, doubleLogs.get("FollowPath/rotationControllerOutput"), 1e-9);
        assertEquals(expectedOmega, doubleLogs.get("FollowPath/outputOmegaRadPerSec"), 1e-9);
        assertEquals(0.0, doubleLogs.get("FollowPath/rotationOverrideOmegaRadPerSec"), 1e-9);
        assertFalse(booleanLogs.get("FollowPath/rotationOverrideActive"));
        assertFalse(booleanLogs.get("FollowPath/rotationOverrideBypassesConstraints"));
    }

    @Test
    void rotationOverrideSupplierRunsEachExecute() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        AtomicInteger supplierCalls = new AtomicInteger(0);
        FollowPath.overrideRotation(() -> {
            supplierCalls.incrementAndGet();
            return 0.25;
        });

        FollowPath command = createCommand(createDegenerateRotationPath(1.0), robot);
        command.initialize();
        runExecute(command, robot);
        runExecute(command, robot);

        assertEquals(2, supplierCalls.get(), "Override supplier should be sampled every execute cycle");
        assertEquals(0.25, robot.getRobotRelativeSpeeds().omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void limitedRotationOverrideRespectsPathConstraints() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Map<String, Double> doubleLogs = new HashMap<>();
        Map<String, Boolean> booleanLogs = new HashMap<>();
        FollowPath.setDoubleLoggingConsumer((Pair<String, Double> pair) -> doubleLogs.put(pair.getFirst(), pair.getSecond()));
        FollowPath.setBooleanLoggingConsumer((Pair<String, Boolean> pair) -> booleanLogs.put(pair.getFirst(), pair.getSecond()));
        FollowPath.overrideRotation(
            () -> 10.0,
            FollowPath.RotationOverrideBehavior.RESPECT_CONSTRAINTS
        );

        FollowPath command = createCommand(createDegenerateRotationPath(1.0), robot);
        command.initialize();
        runExecute(command, robot);

        double expectedLimitedOmega = Math.toRadians(TEST_GLOBAL_CONSTRAINTS.getMaxAccelerationDegPerSec2()) * 0.02;
        assertEquals(expectedLimitedOmega, robot.getRobotRelativeSpeeds().omegaRadiansPerSecond, 1e-9);
        assertEquals(10.0, doubleLogs.get("FollowPath/rotationOverrideOmegaRadPerSec"), 1e-9);
        assertEquals(expectedLimitedOmega, doubleLogs.get("FollowPath/outputOmegaRadPerSec"), 1e-9);
        assertTrue(booleanLogs.get("FollowPath/rotationOverrideActive"));
        assertFalse(booleanLogs.get("FollowPath/rotationOverrideBypassesConstraints"));
    }

    @Test
    void defaultRotationOverrideSkipsPathFollowerOmegaLimits() {
        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
        Map<String, Double> doubleLogs = new HashMap<>();
        Map<String, Boolean> booleanLogs = new HashMap<>();
        FollowPath.setDoubleLoggingConsumer((Pair<String, Double> pair) -> doubleLogs.put(pair.getFirst(), pair.getSecond()));
        FollowPath.setBooleanLoggingConsumer((Pair<String, Boolean> pair) -> booleanLogs.put(pair.getFirst(), pair.getSecond()));
        FollowPath.overrideRotation(() -> 10.0);

        FollowPath command = createCommand(createDegenerateRotationPath(1.0), robot);
        command.initialize();
        runExecute(command, robot);

        assertEquals(0.0, robot.getRobotRelativeSpeeds().vxMetersPerSecond, 1e-9);
        assertEquals(0.0, robot.getRobotRelativeSpeeds().vyMetersPerSecond, 1e-9);
        assertEquals(10.0, robot.getRobotRelativeSpeeds().omegaRadiansPerSecond, 1e-9);
        assertEquals(10.0, doubleLogs.get("FollowPath/rotationOverrideOmegaRadPerSec"), 1e-9);
        assertEquals(10.0, doubleLogs.get("FollowPath/rotationControllerOutput"), 1e-9);
        assertEquals(10.0, doubleLogs.get("FollowPath/outputOmegaRadPerSec"), 1e-9);
        assertTrue(booleanLogs.get("FollowPath/rotationOverrideActive"));
        assertTrue(booleanLogs.get("FollowPath/rotationOverrideBypassesConstraints"));
    }

    @Test
    void clearRotationOverrideRestoresNormalRotationForNewCommands() {
        MutableRobot overrideRobot = new MutableRobot(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        FollowPath.setTimestampSupplier(overrideRobot::getTimestampSeconds);
        FollowPath.overrideRotation(() -> 10.0);
        FollowPath overrideCommand = createCommand(createDegenerateRotationPath(1.0), overrideRobot);
        overrideCommand.initialize();
        runExecute(overrideCommand, overrideRobot);
        assertEquals(10.0, overrideRobot.getRobotRelativeSpeeds().omegaRadiansPerSecond, 1e-9);

        FollowPath.clearRotationOverride();

        MutableRobot normalRobot = new MutableRobot(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        FollowPath.setTimestampSupplier(normalRobot::getTimestampSeconds);
        FollowPath normalCommand = createCommand(createDegenerateRotationPath(1.0), normalRobot);
        normalCommand.initialize();
        runExecute(normalCommand, normalRobot);

        double expectedOmega = 5.0 * Math.toRadians(1.0);
        assertEquals(expectedOmega, normalRobot.getRobotRelativeSpeeds().omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void shouldMirrorVerticallyWhenSupplierReturnsTrue() {
        FlippingUtil.FieldSymmetry originalSymmetryType = FlippingUtil.symmetryType;
        try {
            FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;

            MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
            FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
            Translation2d[][] loggedPathTranslations = new Translation2d[1][];
            FollowPath.setTranslationListLoggingConsumer((Pair<String, Translation2d[]> value) -> {
                if ("FollowPath/pathTranslations".equals(value.getFirst())) {
                    loggedPathTranslations[0] = value.getSecond();
                }
            });

            Path path = new Path(
                new Path.TranslationTarget(new Translation2d(1.0, 2.0)),
                new Path.TranslationTarget(new Translation2d(3.0, 4.0))
            );

            FollowPath command = new FollowPath.Builder(
                new TestSubsystem(),
                robot::getPose,
                robot::getRobotRelativeSpeeds,
                robot::setRobotRelativeSpeeds,
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(0.0, 0.0, 0.0)
            ).withShouldMirror(() -> true).build(path);
            command.initialize();

            assertNotNull(loggedPathTranslations[0], "Expected path translation list to be logged during initialization");
            assertEquals(1.0, loggedPathTranslations[0][0].getX(), 1e-9);
            assertEquals(FlippingUtil.fieldSizeY - 2.0, loggedPathTranslations[0][0].getY(), 1e-9);
            assertEquals(3.0, loggedPathTranslations[0][1].getX(), 1e-9);
            assertEquals(FlippingUtil.fieldSizeY - 4.0, loggedPathTranslations[0][1].getY(), 1e-9);
        } finally {
            FlippingUtil.symmetryType = originalSymmetryType;
        }
    }

    @Test
    void shouldMirrorDoesNothingWhenSupplierReturnsFalse() {
        FlippingUtil.FieldSymmetry originalSymmetryType = FlippingUtil.symmetryType;
        try {
            FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;

            MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
            FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
            Translation2d[][] loggedPathTranslations = new Translation2d[1][];
            FollowPath.setTranslationListLoggingConsumer((Pair<String, Translation2d[]> value) -> {
                if ("FollowPath/pathTranslations".equals(value.getFirst())) {
                    loggedPathTranslations[0] = value.getSecond();
                }
            });

            Path path = new Path(
                new Path.TranslationTarget(new Translation2d(1.0, 2.0)),
                new Path.TranslationTarget(new Translation2d(3.0, 4.0))
            );

            FollowPath command = new FollowPath.Builder(
                new TestSubsystem(),
                robot::getPose,
                robot::getRobotRelativeSpeeds,
                robot::setRobotRelativeSpeeds,
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(0.0, 0.0, 0.0)
            ).withShouldMirror(() -> false).build(path);
            command.initialize();

            assertNotNull(loggedPathTranslations[0], "Expected path translation list to be logged during initialization");
            assertEquals(1.0, loggedPathTranslations[0][0].getX(), 1e-9);
            assertEquals(2.0, loggedPathTranslations[0][0].getY(), 1e-9);
            assertEquals(3.0, loggedPathTranslations[0][1].getX(), 1e-9);
            assertEquals(4.0, loggedPathTranslations[0][1].getY(), 1e-9);
        } finally {
            FlippingUtil.symmetryType = originalSymmetryType;
        }
    }

    @Test
    void shouldFlipUsesRotationalSymmetryEvenIfGlobalSymmetryIsMirrored() {
        FlippingUtil.FieldSymmetry originalSymmetryType = FlippingUtil.symmetryType;
        try {
            FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kMirrored;

            MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
            FollowPath.setTimestampSupplier(robot::getTimestampSeconds);
            Translation2d[][] loggedPathTranslations = new Translation2d[1][];
            FollowPath.setTranslationListLoggingConsumer((Pair<String, Translation2d[]> value) -> {
                if ("FollowPath/pathTranslations".equals(value.getFirst())) {
                    loggedPathTranslations[0] = value.getSecond();
                }
            });

            Path path = new Path(
                new Path.TranslationTarget(new Translation2d(1.0, 2.0)),
                new Path.TranslationTarget(new Translation2d(3.0, 4.0))
            );

            FollowPath command = new FollowPath.Builder(
                new TestSubsystem(),
                robot::getPose,
                robot::getRobotRelativeSpeeds,
                robot::setRobotRelativeSpeeds,
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(0.0, 0.0, 0.0)
            ).withShouldFlip(() -> true).build(path);
            command.initialize();

            assertNotNull(loggedPathTranslations[0], "Expected path translation list to be logged during initialization");
            assertEquals(FlippingUtil.fieldSizeX - 1.0, loggedPathTranslations[0][0].getX(), 1e-9);
            assertEquals(FlippingUtil.fieldSizeY - 2.0, loggedPathTranslations[0][0].getY(), 1e-9);
            assertEquals(FlippingUtil.fieldSizeX - 3.0, loggedPathTranslations[0][1].getX(), 1e-9);
            assertEquals(FlippingUtil.fieldSizeY - 4.0, loggedPathTranslations[0][1].getY(), 1e-9);
            assertEquals(FlippingUtil.FieldSymmetry.kMirrored, FlippingUtil.symmetryType, "flip() should restore prior symmetry type");
        } finally {
            FlippingUtil.symmetryType = originalSymmetryType;
        }
    }

    @Test
    void warmupCommandIsBuiltAndRunsWhileDisabled() {
        // buildWarmupCommand decorates with withTimeout(...), which constructs a WaitCommand/Timer
        // that loads WPILib HAL natives. Skip when those natives are unavailable (e.g. headless CI
        // without extracted natives); the test still runs in a full WPILib test environment.
        assumeTrue(isWpilibHalRuntimeAvailable(), "WPILib HAL runtime is not available");

        MutableRobot robot = new MutableRobot(new Pose2d(0.0, 0.0, new Rotation2d()));
        AtomicInteger driveConsumerCalls = new AtomicInteger(0);
        AtomicInteger poseResetCalls = new AtomicInteger(0);

        edu.wpi.first.wpilibj2.command.Command warmup = new FollowPath.Builder(
            new TestSubsystem(),
            robot::getPose,
            robot::getRobotRelativeSpeeds,
            speeds -> driveConsumerCalls.incrementAndGet(),
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(0.0, 0.0, 0.0)
        ).withPoseReset(pose -> poseResetCalls.incrementAndGet())
            .buildWarmupCommand();

        assertNotNull(warmup, "Warm-up command must be constructible from a builder");
        assertTrue(warmup.runsWhenDisabled(),
            "Warm-up command must run while the robot is disabled so it can warm up in robotInit");
        // The warm-up overrides the builder's drive consumer and pose-reset consumer, so neither
        // is captured by the returned command. Building it must not touch the real robot wiring.
        assertEquals(0, driveConsumerCalls.get(), "Building warm-up must not invoke the drive consumer");
        assertEquals(0, poseResetCalls.get(), "Building warm-up must not invoke the pose-reset consumer");
    }

    private static FollowPath createCommand(Path path, MutableRobot robot) {
        return createCommand(path, robot, false);
    }

    private static FollowPath createCommand(Path path, MutableRobot robot, boolean useTRatioBasedTranslationHandoffs) {
        return new FollowPath.Builder(
            new TestSubsystem(),
            robot::getPose,
            robot::getRobotRelativeSpeeds,
            robot::setRobotRelativeSpeeds,
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(0.0, 0.0, 0.0)
        ).withTRatioBasedTranslationHandoffs(useTRatioBasedTranslationHandoffs)
            .build(path);
    }

    private static Path createDegenerateRotationPath(double targetDegrees) {
        return new Path(
            new Path.TranslationTarget(new Translation2d(0.0, 0.0)),
            new Path.RotationTarget(Rotation2d.fromDegrees(targetDegrees), 1.0),
            new Path.TranslationTarget(new Translation2d(0.0, 0.0))
        );
    }

    private static final class TestSubsystem implements Subsystem {}

    private static boolean isWpilibHalRuntimeAvailable() {
        try {
            ClassLoader classLoader = FollowPathTest.class.getClassLoader();
            Class.forName("edu.wpi.first.hal.NotifierJNI", false, classLoader);
            return isLibraryOnPath("wpiHaljni") && isLibraryOnPath("wpiutil");
        } catch (ClassNotFoundException exception) {
            return false;
        }
    }

    private static boolean isLibraryOnPath(String libraryName) {
        String mappedLibraryName = System.mapLibraryName(libraryName);
        String[] libraryPaths = System.getProperty("java.library.path", "")
            .split(java.io.File.pathSeparator);
        for (String libraryPath : libraryPaths) {
            if (!libraryPath.isBlank()
                && java.nio.file.Files.exists(java.nio.file.Path.of(libraryPath, mappedLibraryName))) {
                return true;
            }
        }
        return false;
    }

    private static String newEventKey(String prefix) {
        return prefix + "-" + EVENT_KEY_COUNTER.incrementAndGet();
    }

    private static boolean areSpeedsNearZero(ChassisSpeeds speeds, double epsilon) {
        return Math.abs(speeds.vxMetersPerSecond) <= epsilon &&
            Math.abs(speeds.vyMetersPerSecond) <= epsilon &&
            Math.abs(speeds.omegaRadiansPerSecond) <= epsilon;
    }

    private static void runExecute(FollowPath command, MutableRobot robot) {
        robot.advanceTime(0.02);
        command.execute();
    }

    private static final class MutableRobot {
        private Pose2d pose;
        private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
        private double timestampSeconds = 0.0;

        private MutableRobot(Pose2d pose) {
            this.pose = pose;
        }

        private Pose2d getPose() {
            return pose;
        }

        private void setPose(Pose2d pose) {
            this.pose = pose;
        }

        private ChassisSpeeds getRobotRelativeSpeeds() {
            return robotRelativeSpeeds;
        }

        private void setRobotRelativeSpeeds(ChassisSpeeds robotRelativeSpeeds) {
            this.robotRelativeSpeeds = robotRelativeSpeeds;
        }

        private double getTimestampSeconds() {
            return timestampSeconds;
        }

        private void advanceTime(double dtSeconds) {
            timestampSeconds += dtSeconds;
        }
    }
}
