package frc.robot.lib.BLine;

import java.util.OptionalDouble;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;

/** Tank travel direction, terminal braking/alignment, and coupled chassis limits. */
final class TankController {
    enum Phase { FOLLOW, BRAKE, ALIGN, ALIGN_BRAKE, DONE }
    record Target(double forward, double heading, boolean steer, boolean phaseChanged) {}

    // Motion below these sensor-level thresholds counts as stopped, provided
    // the slew-limited command itself has reached zero. Units: m/s and rad/s.
    static final double STOPPED_VELOCITY = 0.02;
    private Phase phase = Phase.FOLLOW;
    private TankRateLimiter.Velocity command;

    TankController(ChassisVelocities initialMeasured) {
        command = new TankRateLimiter.Velocity(initialMeasured.vx, initialMeasured.omega);
    }

    Target target(double fieldVx, double fieldVy, Pose2d pose, ChassisVelocities measured,
            boolean atFinalPosition, boolean rolling, OptionalDouble finalHeading,
            double headingTolerance, DriveDirection direction) {
        Phase previous = phase;
        boolean stopped = Math.abs(command.forward()) < 1e-8 && Math.abs(command.omega()) < 1e-8
            && Math.hypot(measured.vx, measured.vy) <= STOPPED_VELOCITY
            && Math.abs(measured.omega) <= STOPPED_VELOCITY;
        double heading = finalHeading.orElse(pose.getRotation().getRadians());
        double error = MathUtil.angleModulus(heading - pose.getRotation().getRadians());
        if (phase == Phase.FOLLOW && atFinalPosition && !rolling) phase = Phase.BRAKE;
        if (phase == Phase.BRAKE && stopped) phase = atFinalPosition ? Phase.ALIGN : Phase.FOLLOW;
        if (phase == Phase.ALIGN) {
            if (!atFinalPosition) phase = Phase.BRAKE;
            else if (Math.abs(error) <= headingTolerance) phase = Phase.ALIGN_BRAKE;
        }
        if (phase == Phase.ALIGN_BRAKE && stopped) {
            phase = !atFinalPosition ? Phase.FOLLOW : Math.abs(error) > headingTolerance ? Phase.ALIGN : Phase.DONE;
        }
        double magnitude = Math.hypot(fieldVx, fieldVy);
        boolean follow = phase == Phase.FOLLOW;
        boolean steer = phase == Phase.ALIGN || (follow && magnitude > 1e-9);
        if (follow && magnitude > 1e-9) heading = Math.atan2(fieldVy, fieldVx)
            + (direction == DriveDirection.BACKWARD ? Math.PI : 0);
        double forward = follow ? magnitude * (direction == DriveDirection.BACKWARD ? -1 : 1) : 0;
        return new Target(forward, heading, steer, previous != phase);
    }

    TankRateLimiter.Result limit(Target target, double omega, TankRateLimiter.Limits limits,
            double dt, DriveDirection direction) {
        var result = TankRateLimiter.limit(command, new TankRateLimiter.Velocity(target.forward, omega), limits, dt, direction);
        command = result.velocity();
        return result;
    }

    void overrideOmega(double omega) { command = new TankRateLimiter.Velocity(command.forward(), omega); }
    TankRateLimiter.Velocity commandedVelocity() { return command; }
    boolean finished() { return phase == Phase.DONE; }
}
