package frc.robot.lib.BLine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisRateLimiter {
    public static ChassisSpeeds limit(
        ChassisSpeeds desiredFieldRelativeSpeeds, 
        ChassisSpeeds lastFieldRelativeSpeeds,
        double dt,
        double maxTranslationalAccelerationMetersPerSec2,
        double maxAngularAccelerationRadiansPerSec2,
        double maxTranslationalVelocityMetersPerSec,
        double maxAngularVelocityRadiansPerSec
    ) {
        if (maxTranslationalVelocityMetersPerSec > 0 && maxAngularVelocityRadiansPerSec > 0) {
            double desiredVelocity = Math.hypot(
                desiredFieldRelativeSpeeds.vxMetersPerSecond,
                desiredFieldRelativeSpeeds.vyMetersPerSecond
            );
            if (desiredVelocity > maxTranslationalVelocityMetersPerSec) {
                double scaleFactor = maxTranslationalVelocityMetersPerSec / desiredVelocity;
                desiredFieldRelativeSpeeds = new ChassisSpeeds(
                    desiredFieldRelativeSpeeds.vxMetersPerSecond * scaleFactor,
                    desiredFieldRelativeSpeeds.vyMetersPerSecond * scaleFactor,
                    desiredFieldRelativeSpeeds.omegaRadiansPerSecond
                );
            }
            desiredFieldRelativeSpeeds.omegaRadiansPerSecond = MathUtil.clamp(
                desiredFieldRelativeSpeeds.omegaRadiansPerSecond,
                -maxAngularVelocityRadiansPerSec,
                maxAngularVelocityRadiansPerSec
            );
        }

        if (dt <= 0) {
            return desiredFieldRelativeSpeeds;
        }
        
        double desiredAcceleration = Math.hypot(
            desiredFieldRelativeSpeeds.vxMetersPerSecond - lastFieldRelativeSpeeds.vxMetersPerSecond,
            desiredFieldRelativeSpeeds.vyMetersPerSecond - lastFieldRelativeSpeeds.vyMetersPerSecond
        ) / dt;

        double obtainableAcceleration = MathUtil.clamp(
            desiredAcceleration,
            0,
            maxTranslationalAccelerationMetersPerSec2
        );

        double theta = Math.atan2(
            desiredFieldRelativeSpeeds.vyMetersPerSecond - lastFieldRelativeSpeeds.vyMetersPerSecond,
            desiredFieldRelativeSpeeds.vxMetersPerSecond - lastFieldRelativeSpeeds.vxMetersPerSecond
        );

        double desiredOmegaAcceleration = (desiredFieldRelativeSpeeds.omegaRadiansPerSecond - lastFieldRelativeSpeeds.omegaRadiansPerSecond) / dt;

        double obtainableOmegaAcceleration = MathUtil.clamp(
            desiredOmegaAcceleration,
            -maxAngularAccelerationRadiansPerSec2,
            maxAngularAccelerationRadiansPerSec2
        );

        return new ChassisSpeeds(
            lastFieldRelativeSpeeds.vxMetersPerSecond + Math.cos(theta) * obtainableAcceleration * dt,
            lastFieldRelativeSpeeds.vyMetersPerSecond + Math.sin(theta) * obtainableAcceleration * dt,
            lastFieldRelativeSpeeds.omegaRadiansPerSecond + obtainableOmegaAcceleration * dt
        );
    }
}
