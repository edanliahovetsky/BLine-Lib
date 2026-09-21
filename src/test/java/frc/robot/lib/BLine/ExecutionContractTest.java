package frc.robot.lib.BLine;

import static org.junit.jupiter.api.Assertions.*;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.Subsystem;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Execution contracts that historically failed on reuse, mutation or coroutine ownership. */
class ExecutionContractTest {
    private final Scheduler scheduler = Scheduler.getDefault();
    private double time;

    private static class Robot {
        final Subsystem subsystem = new Subsystem() {};
        final Mechanism mechanism = new Mechanism() {};
        Pose2d pose = new Pose2d();
        ChassisVelocities output = new ChassisVelocities();
        int resets;
        void reset(Pose2d value) { resets++; pose = value; }
    }

    @BeforeEach void setup() {
        Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(4, 4, 720, 1440, .05, 2, .2));
        Follower.setTimestampSupplier(() -> time);
        FollowPath.clearPendingEventTriggers();
        FollowPathV2.clearPendingEventTriggers();
    }

    @AfterEach void cleanup() {
        scheduler.cancelAll();
        FollowPath.clearPendingEventTriggers();
        FollowPathV2.clearPendingEventTriggers();
        Follower.setTimestampSupplier(null);
        Follower.setDoubleLoggingConsumer(value -> {});
    }

    private void cycle() { time += .02; scheduler.run(); }

    private FollowPath.Builder v3(Robot robot) {
        return new FollowPath.Builder(DriveType.SWERVE, robot.mechanism, () -> robot.pose, robot::reset,
            ChassisVelocities::new, value -> robot.output = value,
            new PIDController(3, 0, 0), new PIDController(3, 0, 0), new PIDController(1, 0, 0));
    }

    private FollowPathV2.Builder v2(Robot robot) {
        return new FollowPathV2.Builder(DriveType.SWERVE, robot.subsystem, () -> robot.pose, robot::reset,
            ChassisVelocities::new, value -> robot.output = value,
            new PIDController(3, 0, 0), new PIDController(3, 0, 0), new PIDController(1, 0, 0));
    }

    @Test void finalEventRunsAfterV3FollowerAndItsParentComplete() {
        Robot robot = new Robot();
        AtomicInteger starts = new AtomicInteger();
        Command event = Command.noRequirements(co -> { starts.incrementAndGet(); co.park(); }).named("Event");
        FollowPath.registerEventTrigger("final-v3", event);
        Path path = new Path(new Path.TranslationTarget(0, 0),
            new Path.EventTrigger(1, "final-v3"), new Path.TranslationTarget(1, 0));
        FollowPath follower = v3(robot).build(path);
        Command auto = Command.noRequirements(co -> co.await(follower)).named("Auto");
        scheduler.schedule(auto);
        cycle();
        assertTrue(scheduler.isScheduledOrRunning(follower));
        robot.pose = new Pose2d(1, 0, Rotation2d.ZERO);
        cycle();
        cycle();
        assertFalse(scheduler.isScheduledOrRunning(follower));
        assertFalse(scheduler.isScheduledOrRunning(auto));
        assertEquals(1, starts.get());
        assertTrue(scheduler.isScheduledOrRunning(event), "Event must not be a child of the completed path or auto");
        scheduler.cancel(event);
        assertFalse(scheduler.isScheduledOrRunning(event));
    }

    @Test void cancellationDropsOnlyTheCancelledExecutionsPendingEvents() {
        AtomicInteger cancelledCount = new AtomicInteger();
        AtomicInteger survivingCount = new AtomicInteger();
        FollowPath.registerEventTrigger("cancelled-v3", cancelledCount::incrementAndGet);
        FollowPath.registerEventTrigger("surviving-v3", survivingCount::incrementAndGet);
        FollowPath cancelled = v3(new Robot()).build(new Path(new Path.EventTrigger(0, "cancelled-v3"), new Path.TranslationTarget(2, 0)));
        FollowPath surviving = v3(new Robot()).build(new Path(new Path.EventTrigger(0, "surviving-v3"), new Path.TranslationTarget(2, 0)));
        scheduler.schedule(cancelled);
        scheduler.schedule(surviving);
        cycle();
        assertTrue(scheduler.isRunning(cancelled));
        assertEquals(0, cancelledCount.get());
        assertEquals(0, survivingCount.get());
        // Both runs enqueue before the next event-loop poll. Cancel only the first execution.
        scheduler.cancel(cancelled);
        cycle();
        cycle();
        assertEquals(0, cancelledCount.get());
        assertEquals(1, survivingCount.get());
        assertTrue(scheduler.isScheduledOrRunning(surviving));
    }

    @Test void perCommandOptionsAndDynamicFlipDoNotLeakOrAccumulate() {
        Robot robot = new Robot();
        AtomicBoolean flip = new AtomicBoolean(true);
        AtomicInteger samples = new AtomicInteger();
        Path path = new Path(new Path.Waypoint(new Pose2d(1, 2, Rotation2d.fromDegrees(30))), new Path.TranslationTarget(3, 2));
        var builder = v2(robot).withShouldFlip(() -> { samples.incrementAndGet(); return flip.get(); });
        var reset = builder.build(path).withPoseReset().withShouldMirror(() -> true);
        var plain = builder.build(path).withShouldFlip(() -> false);
        reset.initialize();
        Pose2d firstReset = robot.pose;
        reset.end(true);
        reset.initialize();
        assertEquals(firstReset, robot.pose, "Repeated execution must not apply another reflection");
        reset.end(true);
        flip.set(false);
        reset.initialize();
        assertEquals(1, robot.pose.getX(), 1e-9);
        assertEquals(FlippingUtil.fieldSizeY - 2, robot.pose.getY(), 1e-9);
        reset.end(true);
        int resets = robot.resets;
        plain.initialize();
        assertEquals(resets, robot.resets, "Pose reset is enabled per command, never inherited by the builder");
        assertEquals(3, samples.get(), "Policies are sampled once per execution and overrides replace the default");
        assertFalse(path.isFlipped());
        assertFalse(path.isMirrored());
        plain.end(true);
    }

    @Test void mutatedPathIsValidatedAtExecutionAndCanBeRepairedAndReused() {
        Robot robot = new Robot();
        Path path = new Path(new Path.TranslationTarget(0, 0), new Path.TranslationTarget(2, 0));
        var command = v2(robot).build(path).withPoseReset();
        path.setElement(1, new Path.RotationTarget(Rotation2d.ZERO, .5));
        command.initialize();
        command.execute();
        assertTrue(command.isFinished());
        assertEquals(0, robot.resets, "Invalid paths must not reset odometry");
        assertEquals(0, robot.output.vx);
        command.end(true);
        path.setElement(1, new Path.Waypoint((Path.TranslationTarget) null, new Path.RotationTarget(Rotation2d.ZERO, 1)));
        assertDoesNotThrow(command::initialize, "Incomplete mutable waypoints must report validation failure before copying");
        assertTrue(command.isFinished());
        assertEquals(0, robot.resets);
        command.end(true);
        path.setElement(1, new Path.TranslationTarget(4, 0));
        command.initialize();
        time += .02;
        command.execute();
        assertFalse(command.isFinished());
        assertEquals(4, command.getRemainingPathDistanceMeters(), 1e-9);
        assertEquals(1, robot.resets);
        command.end(true);
    }

    @Test void invalidResolvedLimitsCannotResetPoseOrFireEvents() {
        for (double acceleration : new double[] {-1, 0, Double.NaN, Double.POSITIVE_INFINITY}) {
            Robot robot = new Robot();
            AtomicInteger events = new AtomicInteger();
            FollowPathV2.registerEventTrigger("invalid-limits", events::incrementAndGet);
            Path path = new Path(new Path.PathConstraints().setMaxAccelerationMetersPerSec2(acceleration),
                new Path.TranslationTarget(0, 0), new Path.EventTrigger(0, "invalid-limits"), new Path.TranslationTarget(2, 0));
            var command = v2(robot).build(path).withPoseReset();
            command.initialize();
            time += .02;
            command.execute();
            org.wpilib.command2.CommandScheduler.getInstance().getDefaultButtonLoop().poll();
            assertTrue(command.isFinished());
            assertEquals(0, robot.resets);
            assertEquals(0, events.get());
            assertEquals(0, robot.output.vx);
            command.end(true);
        }
    }

    @Test void singleWaypointInterpolatesItsEndingHeadingFromTheMeasuredStart() {
        Robot robot = new Robot();
        java.util.Map<String, Double> samples = new java.util.HashMap<>();
        Follower.setDoubleLoggingConsumer(value -> samples.put(value.getFirst(), value.getSecond()));
        var command = v2(robot).build(new Path(new Path.Waypoint(new Pose2d(2, 0, Rotation2d.fromDegrees(90)))));
        command.initialize();
        time += .02;
        command.execute();
        assertEquals(0, samples.get("FollowPath/targetRotationDeg"), 1e-9);
        robot.pose = new Pose2d(1, 0, Rotation2d.ZERO);
        time += .02;
        command.execute();
        assertEquals(45, samples.get("FollowPath/targetRotationDeg"), 1e-9);
        command.end(true);
    }

    @Test void shortPathSkipsResetAndUsesMeasuredPoseForLeadingEventAndRotation() {
        Robot robot = new Robot();
        robot.pose = new Pose2d(4, 2, Rotation2d.ZERO);
        AtomicInteger events = new AtomicInteger();
        FollowPathV2.registerEventTrigger("ghost-start", events::incrementAndGet);
        Path path = new Path(new Path.EventTrigger(0, "ghost-start"),
            new Path.RotationTarget(Rotation2d.fromDegrees(90), .5), new Path.TranslationTarget(6, 2));
        var command = v2(robot).build(path).withPoseReset();
        command.initialize();
        time += .02;
        command.execute();
        org.wpilib.command2.CommandScheduler.getInstance().getDefaultButtonLoop().poll();
        assertEquals(0, robot.resets);
        assertEquals(2, command.getRemainingPathDistanceMeters(), 1e-9);
        assertEquals(1, events.get());
        assertEquals(3, path.getPathElements().size(), "Execution origin must not be appended to the authored path");
        command.end(true);
    }
}
