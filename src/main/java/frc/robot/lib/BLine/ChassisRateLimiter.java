package frc.robot.lib.BLine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A utility class that limits the rate of change of chassis speeds for smooth motion control.
 * 
 * <p>This class provides acceleration limiting for both translational and rotational velocities
 * in field-relative coordinates. It ensures that the robot's movement remains within physical
 * constraints and prevents sudden changes in velocity that could cause wheel slip or mechanical stress.
 * 
 * <p>The limiter operates in two phases:
 * <ol>
 *   <li>Velocity limiting: Clamps the desired speeds to maximum allowed velocities</li>
 *   <li>Acceleration limiting: Limits the rate of change from the previous velocity state</li>
 * </ol>
 * 
 * <p>Example usage:
 * <pre>{@code
 * ChassisSpeeds limited = ChassisRateLimiter.limit(
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
 * @see edu.wpi.first.math.kinematics.ChassisSpeeds
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
     *                                              per second. Set to 0 or negative to disable velocity limiting.
     * @param maxAngularVelocityRadiansPerSec The maximum allowed angular velocity in radians per second.
     *                                         Set to 0 or negative to disable velocity limiting.
     * @return A new {@link ChassisSpeeds} object with velocities constrained to the specified limits.
     *         The returned speeds will not exceed the maximum velocities and the change from
     *         {@code lastSpeeds} will not exceed the maximum accelerations.
     */
    public static ChassisSpeeds limit(
        ChassisSpeeds desiredSpeeds, 
        ChassisSpeeds lastSpeeds,
        double dt,
        double maxTranslationalAccelerationMetersPerSec2,
        double maxAngularAccelerationRadiansPerSec2,
        double maxTranslationalVelocityMetersPerSec,
        double maxAngularVelocityRadiansPerSec
    ) {
        if (maxTranslationalVelocityMetersPerSec > 0 && maxAngularVelocityRadiansPerSec > 0) {
            double desiredVelocity = Math.hypot(
                desiredSpeeds.vxMetersPerSecond,
                desiredSpeeds.vyMetersPerSecond
            );
            if (desiredVelocity > maxTranslationalVelocityMetersPerSec) {
                double scaleFactor = maxTranslationalVelocityMetersPerSec / desiredVelocity;
                desiredSpeeds = new ChassisSpeeds(
                    desiredSpeeds.vxMetersPerSecond * scaleFactor,
                    desiredSpeeds.vyMetersPerSecond * scaleFactor,
                    desiredSpeeds.omegaRadiansPerSecond
                );
            }
            desiredSpeeds.omegaRadiansPerSecond = MathUtil.clamp(
                desiredSpeeds.omegaRadiansPerSecond,
                -maxAngularVelocityRadiansPerSec,
                maxAngularVelocityRadiansPerSec
            );
        }

        if (dt <= 0) {
            return desiredSpeeds;
        }
        
        double desiredAcceleration = Math.hypot(
            desiredSpeeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond,
            desiredSpeeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond
        ) / dt;

        double obtainableAcceleration = MathUtil.clamp(
            desiredAcceleration,
            0,
            maxTranslationalAccelerationMetersPerSec2
        );

        double theta = Math.atan2(
            desiredSpeeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond,
            desiredSpeeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond
        );

        double desiredOmegaAcceleration = (desiredSpeeds.omegaRadiansPerSecond - lastSpeeds.omegaRadiansPerSecond) / dt;

        double obtainableOmegaAcceleration = MathUtil.clamp(
            desiredOmegaAcceleration,
            -maxAngularAccelerationRadiansPerSec2,
            maxAngularAccelerationRadiansPerSec2
        );

        return new ChassisSpeeds(
            lastSpeeds.vxMetersPerSecond + Math.cos(theta) * obtainableAcceleration * dt,
            lastSpeeds.vyMetersPerSecond + Math.sin(theta) * obtainableAcceleration * dt,
            lastSpeeds.omegaRadiansPerSecond + obtainableOmegaAcceleration * dt
        );
    }
}
