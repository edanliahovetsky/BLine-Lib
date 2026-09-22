package frc.robot.lib.BLine.following;

import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * A utility class that limits the rate of change of chassis speeds for smooth motion control.
 * 
 * <p>This class provides acceleration limiting for both translational and rotational velocities
 * in field-relative coordinates. The robot's wheel/motor control remains responsible for
 * achieving these requested chassis velocities.
 * 
 * <p>The limiter operates in two phases:
 * <ol>
 *   <li>Velocity limiting: Clamps the desired speeds to maximum allowed velocities</li>
 *   <li>Acceleration limiting: Limits the rate of change from the previous velocity state</li>
 * </ol>
 * 
 * <p>Example usage:
 * <pre>{@code
 * ChassisVelocities limited = ChassisRateLimiter.limit(
 *     desiredSpeeds,
 *     lastSpeeds,
 *     0.02,  // 20ms loop time
 *     4.0,   // max translational acceleration (m/s²)
 *     8.0,   // max angular acceleration (rad/s²)
 *     5.0,   // max translational velocity (m/s)
 *     Math.PI * 2  // max angular velocity (rad/s)
 * );
 * }</pre>
 * 
 * @see org.wpilib.math.kinematics.ChassisVelocities
 */
public class ChassisRateLimiter {
    
    /**
     * Limits the chassis speeds to respect both velocity and acceleration constraints.
     * 
     * <p>This method first applies velocity limits to clamp the desired speeds, then applies
     * acceleration limits based on the time delta and previous speeds. The acceleration limiting
     * preserves the direction of the desired velocity change while constraining its magnitude.
     * 
     * <p>If {@code dt <= 0}, only velocity limiting is applied and acceleration limiting is skipped.
     * 
     * @param desiredSpeeds The target chassis speeds to achieve.
     *                       This object may be modified if velocity limiting is applied.
     * @param lastSpeeds The chassis speeds from the previous control loop iteration.
     * @param dt The time delta since the last call, in seconds. Used for acceleration calculations.
     *           If zero or negative, only velocity limiting is applied.
     * @param maxTranslationalAccelerationMetersPerSec2 The maximum allowed translational acceleration
     *                                                   in meters per second squared. Must be positive.
     * @param maxAngularAccelerationRadiansPerSec2 The maximum allowed angular acceleration in radians
     *                                              per second squared. Must be positive.
     * @param maxTranslationalVelocityMetersPerSec The maximum allowed translational velocity in meters
     *                                              per second. Both velocity maxima must be positive to enable velocity limiting.
     * @param maxAngularVelocityRadiansPerSec The maximum allowed angular velocity in radians per second.
     *                                         Both velocity maxima must be positive to enable velocity limiting.
     * @return A new {@link ChassisVelocities} object with velocities constrained to the specified limits.
     *         If the previous speed exceeds a newly lowered maximum, it approaches that maximum
     *         through acceleration limiting instead of jumping immediately to it.
     */
    public static ChassisVelocities limit(
        ChassisVelocities desiredSpeeds,
        ChassisVelocities lastSpeeds,
        double dt,
        double maxTranslationalAccelerationMetersPerSec2,
        double maxAngularAccelerationRadiansPerSec2,
        double maxTranslationalVelocityMetersPerSec,
        double maxAngularVelocityRadiansPerSec
    ) {
        if (maxTranslationalVelocityMetersPerSec > 0 && maxAngularVelocityRadiansPerSec > 0) {
            double desiredVelocity = Math.hypot(
                desiredSpeeds.vx,
                desiredSpeeds.vy
            );
            if (desiredVelocity > maxTranslationalVelocityMetersPerSec) {
                double scaleFactor = maxTranslationalVelocityMetersPerSec / desiredVelocity;
                desiredSpeeds = new ChassisVelocities(
                    desiredSpeeds.vx * scaleFactor,
                    desiredSpeeds.vy * scaleFactor,
                    desiredSpeeds.omega
                );
            }
            desiredSpeeds.omega = Math.clamp(
                desiredSpeeds.omega,
                -maxAngularVelocityRadiansPerSec,
                maxAngularVelocityRadiansPerSec
            );
        }

        if (dt <= 0) {
            return desiredSpeeds;
        }
        
        double desiredAcceleration = Math.hypot(
            desiredSpeeds.vx - lastSpeeds.vx,
            desiredSpeeds.vy - lastSpeeds.vy
        ) / dt;

        double obtainableAcceleration = Math.clamp(
            desiredAcceleration,
            0,
            maxTranslationalAccelerationMetersPerSec2
        );

        double theta = Math.atan2(
            desiredSpeeds.vy - lastSpeeds.vy,
            desiredSpeeds.vx - lastSpeeds.vx
        );

        double desiredOmegaAcceleration = (desiredSpeeds.omega - lastSpeeds.omega) / dt;

        double obtainableOmegaAcceleration = Math.clamp(
            desiredOmegaAcceleration,
            -maxAngularAccelerationRadiansPerSec2,
            maxAngularAccelerationRadiansPerSec2
        );

        return new ChassisVelocities(
            lastSpeeds.vx + Math.cos(theta) * obtainableAcceleration * dt,
            lastSpeeds.vy + Math.sin(theta) * obtainableAcceleration * dt,
            lastSpeeds.omega + obtainableOmegaAcceleration * dt
        );
    }
}
