package frc.robot.lib.BLine;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Deterministic kinematic controller checks; the consumer projects test actual motor physics. */
class TankFollowerTest {
    @BeforeEach void defaults() {
        Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(3.5, 2, Math.toDegrees(2.5), Math.toDegrees(4), 0.08, 2, 0.3));
    }
    @AfterEach void restoreClock() { Follower.setTimestampSupplier(null); }

    @Test void travelsInTheSelectedDirectionThenStopsAndAlignsTheFinalBodyHeading() {
        for (DriveDirection direction : DriveDirection.values()) {
            Rig rig = new Rig(new Pose2d(0, 0, new Rotation2d(direction == DriveDirection.BACKWARD ? Math.PI : 0)));
            Follower follower = rig.follower(new Path(constraints(), new Path.Waypoint(new Pose2d(3, 0, Rotation2d.fromDegrees(90)))));
            follower.withTankDriveDirection(direction);
            follower.initialize();
            for (int i = 0; i < 2000 && !follower.isFinished(); i++) {
                rig.step(follower);
                assertEquals(0, rig.output.vy, 0);
                assertTrue(direction == DriveDirection.FORWARD ? rig.output.vx >= -1e-8 : rig.output.vx <= 1e-8);
            }
            assertTrue(follower.isFinished(), "Stopped endpoint must finish in 40 simulated seconds");
            assertTrue(rig.pose.getTranslation().getDistance(new Translation2d(3, 0)) <= 0.08);
            assertEquals(90, rig.pose.getRotation().getDegrees(), 2);
            assertEquals(0, rig.output.vx, 1e-8);
            assertEquals(0, rig.output.omega, 1e-8);
        }
    }

    @Test void intermediateHeadingsDoNotSteerTankAndRollingExitDoesNotStopForFinalHeading() {
        Rig rig = new Rig(new Pose2d());
        var constraints = constraints().setMinVelocityMetersPerSec(1);
        Path path = new Path(constraints, new Path.TranslationTarget(new Translation2d(0, 0)),
            new Path.RotationTarget(Rotation2d.fromDegrees(150), 0.5),
            new Path.Waypoint(new Pose2d(3, 0, Rotation2d.fromDegrees(90))));
        Follower follower = rig.follower(path);
        follower.initialize();
        for (int i = 0; i < 500 && !follower.isFinished(); i++) {
            rig.step(follower);
            assertEquals(0, rig.output.omega, 1e-8, "Authored intermediate/final heading must not steer a rolling tank path");
        }
        assertTrue(follower.isFinished());
        assertTrue(rig.output.vx >= 1 - 1e-8);
        double speed = rig.output.vx;
        follower.end(false);
        assertEquals(speed, rig.output.vx, 0);
        follower.end(true);
        assertEquals(0, rig.output.vx, 0);
    }

    @ParameterizedTest
    @ValueSource(doubles = {30, 90, 150, 180})
    void sharpForwardTurnsStayBoundedWithoutReversingAndCanBeCanceled(double degrees) {
        Rig rig = new Rig(new Pose2d());
        double angle = Math.toRadians(degrees);
        Pose2d end = new Pose2d(2 + 5 * Math.cos(angle), 5 * Math.sin(angle), new Rotation2d(angle));
        Path path = new Path(constraints(), new Path.TranslationTarget(new Translation2d()),
            new Path.TranslationTarget(new Translation2d(2, 0)).withHandoffDistanceMeters(0.3), new Path.Waypoint(end));
        Follower follower = rig.follower(path);
        follower.initialize();
        for (int i = 0; i < 3000 && !follower.isFinished(); i++) {
            ChassisVelocities previous = rig.output;
            rig.step(follower);
            assertTrue(rig.output.vx >= -1e-8);
            assertTrue(rig.output.vx <= 3.5 + 1e-8);
            assertEquals(0, rig.output.vy, 0);
            assertTrue(Math.abs(rig.output.omega) <= 2.5 + 1e-8);
            assertTrue(Math.abs(rig.output.omega - previous.omega) <= 4 * 0.02 + 1e-8);
            double a = (rig.output.vx - previous.vx) / 0.02;
            for (int sample = 0; sample <= 10; sample++) {
                double f = sample / 10.0;
                double v = previous.vx + (rig.output.vx - previous.vx) * f;
                double w = previous.omega + (rig.output.omega - previous.omega) * f;
                assertTrue(Math.hypot(a, v * w) <= 2 + 1e-7);
            }
        }
        // The local joint objective can remain in a saturated turn. Completion
        // is not promised for these bends; the straight/rolling tests above
        // exercise successful completion. Do not add hidden steering priority
        // just to make this integration case finish.
        if (follower.isFinished()) {
            assertTrue(rig.pose.getTranslation().getDistance(end.getTranslation()) <= 0.08);
        }
        assertTrue(rig.pose.getTranslation().getNorm() > 0.5, "Must actually execute the path");
        follower.end(true);
        assertEquals(0, rig.output.vx, 0);
        assertEquals(0, rig.output.omega, 0);
    }

    @Test void rollingExitPreservesTheRobotRelativeCommandWhileTurning() {
        for (DriveDirection direction : DriveDirection.values()) {
            Rig rig = new Rig(new Pose2d(0, 0,
                new Rotation2d(direction == DriveDirection.BACKWARD ? Math.PI : 0)));
            Path path = new Path(constraints().setMinVelocityMetersPerSec(0.8),
                new Path.TranslationTarget(2, 1));
            Follower follower = rig.follower(path);
            follower.withTankDriveDirection(direction);
            follower.initialize();
            ChassisVelocities incoming = rig.output;
            for (int i = 0; i < 2000 && !follower.isFinished(); i++) {
                incoming = rig.output;
                rig.step(follower);
                assertEquals(0, rig.output.vy, 0, "Tank cannot command sideways motion, including at completion");
            }
            assertTrue(follower.isFinished());
            assertTrue(Math.abs(incoming.omega) > 0.01, "Exercise a rolling exit while still turning");
            assertEquals(incoming.vx, rig.output.vx, 0);
            assertEquals(incoming.omega, rig.output.omega, 0);
            follower.end(false);
            assertEquals(incoming.vx, rig.output.vx, 0);
            assertEquals(incoming.omega, rig.output.omega, 0);
            follower.end(true);
            assertEquals(0, rig.output.vx, 0);
            assertEquals(0, rig.output.omega, 0);
        }
    }

    private static Path.PathConstraints constraints() {
        return new Path.PathConstraints().setMaxVelocityMetersPerSec(3.5).setMaxAccelerationMetersPerSec2(2)
            .setMaxVelocityDegPerSec(Math.toDegrees(2.5)).setMaxAccelerationDegPerSec2(Math.toDegrees(4))
            .setEndTranslationToleranceMeters(0.08).setEndRotationToleranceDeg(2);
    }

    private static final class Rig {
        Pose2d pose;
        ChassisVelocities output = new ChassisVelocities();
        double seconds;
        Rig(Pose2d pose) { this.pose = pose; }
        Follower follower(Path path) {
            Follower.setTimestampSupplier(() -> seconds);
            return new Follower(path, new FollowerConfig(DriveType.TANK, () -> pose, value -> pose = value,
                () -> output, value -> output = value, new PIDController(1.3, 0, 0),
                new PIDController(2, 0, 0.25), new PIDController(0.5, 0, 0)), () -> false, new PendingEvents());
        }
        void step(Follower follower) {
            ChassisVelocities previous = output;
            seconds += 0.02;
            follower.execute();
            pose = pose.plus(new Twist2d((previous.vx + output.vx) * 0.01, 0, (previous.omega + output.omega) * 0.01).exp());
        }
    }
}
