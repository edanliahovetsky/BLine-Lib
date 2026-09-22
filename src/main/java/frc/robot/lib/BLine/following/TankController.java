package frc.robot.lib.BLine.following;

import frc.robot.lib.BLine.path.DriveDirection;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;

/** Converts field-relative guidance into forward travel or final-heading alignment. */
final class TankController {
    record Target(double forward, double heading, boolean steer, boolean stopTranslation) {}

    private TankRateLimiter.Velocity command;

    TankController(ChassisVelocities initialMeasured) {
        command = new TankRateLimiter.Velocity(initialMeasured.vx, initialMeasured.omega);
    }

    Target target(double fieldVx, double fieldVy, Pose2d pose,
            boolean atFinalPosition, OptionalDouble finalHeading,
            double headingTolerance, DriveDirection direction) {
        if (atFinalPosition) {
            double heading = finalHeading.orElse(pose.getRotation().getRadians());
            boolean steer = Math.abs(MathUtil.angleModulus(heading - pose.getRotation().getRadians())) > headingTolerance;
            return new Target(0, heading, steer, true);
        }
        double magnitude = Math.hypot(fieldVx, fieldVy);
        double heading = magnitude > 1e-9 ? Math.atan2(fieldVy, fieldVx)
            + (direction == DriveDirection.BACKWARD ? Math.PI : 0) : pose.getRotation().getRadians();
        return new Target(magnitude * (direction == DriveDirection.BACKWARD ? -1 : 1), heading, magnitude > 1e-9, false);
    }

    TankRateLimiter.Result limit(Target target, double omega, TankRateLimiter.Limits limits,
            double dt, DriveDirection direction) {
        // Arrival is a positional handoff: translation stops immediately while heading
        // continues through the angular limiter. Sensor settling is not a prerequisite.
        if (target.stopTranslation()) command = new TankRateLimiter.Velocity(0, command.omega());
        var result = TankRateLimiter.limit(command, new TankRateLimiter.Velocity(target.forward, omega), limits, dt, direction);
        command = result.velocity();
        return result;
    }

    void stop() { command = new TankRateLimiter.Velocity(0, 0); }
    void overrideOmega(double omega) { command = new TankRateLimiter.Velocity(command.forward(), omega); }
    TankRateLimiter.Velocity commandedVelocity() { return command; }
}
