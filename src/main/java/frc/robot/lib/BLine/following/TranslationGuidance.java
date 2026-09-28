package frc.robot.lib.BLine.following;

import frc.robot.lib.BLine.path.PreparedPath.TranslationLimits;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/** Combines distance and cross-track feedback into the field-relative velocity requested by either drivetrain. */
final class TranslationGuidance {
    record Result(double vx, double vy, double rawSpeed, double clampedSpeed, double speed,
                  boolean minimumApplied, double crossTrackOutput) {}
    private final PIDController distance;
    private final PIDController crossTrack;

    TranslationGuidance(PIDController distance, PIDController crossTrack) {
        this.distance = distance;
        this.crossTrack = crossTrack;
    }

    void reset(double toleranceMeters) {
        distance.reset();
        crossTrack.reset();
        distance.setTolerance(toleranceMeters);
        crossTrack.setTolerance(toleranceMeters);
    }

    Result calculate(Pose2d pose, Translation2d target, double remaining, TranslationProgress.CrossTrack track,
                     TranslationLimits limits, boolean applyMinimum) {
        double bearing = Math.atan2(target.getY() - pose.getY(), target.getX() - pose.getX());
        double raw = -distance.calculate(remaining, 0);
        // Bound the distance request before adding CTE, so it cannot overwhelm that correction.
        double clamped = Math.clamp(raw, -limits.maxVelocityMetersPerSec(), limits.maxVelocityMetersPerSec());
        double speed = minimumMagnitude(clamped, limits.minVelocityMetersPerSec(), limits.maxVelocityMetersPerSec(), remaining, applyMinimum);
        // CTE is intentionally not clamped here. The final vector goes through the chassis limiter.
        // Correct toward the line along the segment normal, independent of the target bearing.
        Translation2d normal = track.leftNormal();
        double correction = normal.getNorm() == 0 ? 0 : -crossTrack.calculate(track.errorMeters(), 0);
        double vx = speed * Math.cos(bearing) - correction * normal.getX();
        double vy = speed * Math.sin(bearing) - correction * normal.getY();
        return new Result(vx, vy, raw, clamped, speed, Math.abs(speed) > Math.abs(clamped) + 1e-9, correction);
    }

    static double minimumMagnitude(double value, double minimum, double maximum, double directionWhenZero, boolean enabled) {
        if (!enabled || minimum <= 0) return value;
        double bounded = maximum > 0 ? Math.min(minimum, maximum) : minimum;
        if (Math.abs(value) >= bounded) return value;
        double sign = Math.signum(value);
        if (sign == 0) sign = Math.signum(directionWhenZero);
        return (sign == 0 ? 1 : sign) * bounded;
    }
}
