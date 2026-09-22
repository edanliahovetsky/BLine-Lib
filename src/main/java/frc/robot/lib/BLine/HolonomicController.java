package frc.robot.lib.BLine;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.robot.lib.BLine.Path.TranslationTargetConstraint;
import frc.robot.lib.BLine.Path.RotationTargetConstraint;

/** Keeps field-relative command history for swerve and mecanum; robot I/O remains robot-relative. */
final class HolonomicController {
    private ChassisVelocities command;

    HolonomicController(ChassisVelocities measured, Rotation2d heading) {
        command = measured.toFieldRelative(heading);
    }

    ChassisVelocities limit(double vx, double vy, double omega, double dt,
            TranslationTargetConstraint translation, RotationTargetConstraint rotation,
            boolean atPosition, boolean atHeading, boolean overrideActive, boolean bypassRotation) {
        command = ChassisRateLimiter.limit(new ChassisVelocities(vx, vy, omega), command, dt,
            translation.maxAccelerationMetersPerSec2(), Math.toRadians(rotation.maxAccelerationDegPerSec2()),
            translation.maxVelocityMetersPerSec(), Math.toRadians(rotation.maxVelocityDegPerSec()));
        if (atPosition) {
            command.vx = 0;
            command.vy = 0;
            if (!overrideActive && atHeading) command.omega = 0;
        }
        if (bypassRotation) command.omega = omega;
        return command;
    }

    ChassisVelocities robotRelativeCommand(Rotation2d heading) { return command.toRobotRelative(heading); }
    void stop() { command = new ChassisVelocities(); }
}
