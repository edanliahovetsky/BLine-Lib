package frc.robot.lib.BLine;

/**
 * Limits signed forward velocity and turn rate, without a wheel model.
 *
 * <p>With linear ramps v(t) and omega(t), the centre's field acceleration is
 * sqrt(a² + (v(t) omega(t))²). At a fixed next omega, the permissible forward
 * accelerations form an interval containing zero. Choose the reachable turn
 * rate closest to the prepared request, then approach the requested forward
 * acceleration from the feasible side of that interval. No angular search,
 * mixed-unit cost or trajectory is involved.
 */
final class TankRateLimiter {
    private static final double EPSILON = 1e-9;
    // Acceleration uncertainty, in m/s²; also capped at 1% for gentle limits.
    private static final double ACCELERATION_PRECISION = 0.02;
    private static final int MAX_BISECTIONS = 16;

    record Velocity(double forward, double omega) {}
    record Limits(double acceleration, double angularAcceleration, double speed, double omega) {}
    record Result(Velocity velocity, boolean recovering) {}

    private TankRateLimiter() {}

    static Result limit(Velocity current, Velocity requested, Limits limits, double dt, DriveDirection direction) {
        if (!(dt > 0) || !Double.isFinite(dt)) return new Result(current, false);
        // A new path/range may lower a limit below the existing motion. Do not
        // invent a feasible initial velocity by clipping it. Recover continuously
        // without increasing the pre-existing speed or cornering excess.
        double acceleration = Math.max(limits.acceleration, Math.abs(current.forward * current.omega));
        double speed = Math.max(limits.speed, Math.abs(current.forward));
        double turnRate = Math.max(limits.omega, Math.abs(current.omega));
        boolean recovering = acceleration > limits.acceleration + EPSILON
            || speed > limits.speed + EPSILON || turnRate > limits.omega + EPSILON;

        double forwardRequest = Math.clamp(requested.forward, -limits.speed, limits.speed);
        double turnRequest = Math.clamp(requested.omega, -limits.omega, limits.omega);
        // A sustained requested turn requires |v * omega| <= A. When the
        // heading PID requests a tighter turn, request the corresponding lower
        // speed as well; otherwise a robot on the cornering boundary cannot
        // begin braking without first freeing lateral acceleration capacity.
        if (Math.abs(turnRequest) > EPSILON) {
            double cornerSpeed = limits.acceleration / Math.abs(turnRequest);
            forwardRequest = Math.copySign(Math.min(Math.abs(forwardRequest), cornerSpeed), forwardRequest);
        }
        // Same vector-budget rule as the prototype: longitudinal acceleration
        // and lateral acceleration (v * omega) are both measured in m/s².
        double a = Math.clamp((forwardRequest - current.forward) / dt, -limits.acceleration, limits.acceleration);
        double magnitude = Math.hypot(a, current.forward * turnRequest);
        double scale = magnitude > limits.acceleration ? limits.acceleration / magnitude : 1;
        double targetForward = current.forward + scale * a * dt;
        double targetOmega = scale * turnRequest;
        double low = Math.max(-turnRate, current.omega - limits.angularAcceleration * dt);
        double high = Math.min(turnRate, current.omega + limits.angularAcceleration * dt);

        double omega = Math.clamp(targetOmega, low, high);
        double forward = Math.clamp(targetForward, -speed, speed);
        // Existing opposite-direction motion may brake; it may not grow.
        if (direction == DriveDirection.FORWARD) forward = Math.max(forward, Math.min(0, current.forward));
        else forward = Math.min(forward, Math.max(0, current.forward));
        double alpha = (omega - current.omega) / dt;
        double requestedAcceleration = (forward - current.forward) / dt;

        // Request preparation guarantees |v0 * targetOmega| <= A. Angular slew
        // keeps omega between that target and the current rate, so holding v0
        // is feasible throughout the step under the recovery envelope. This is
        // already the closest reachable turn rate; searching others cannot
        // improve the turn-priority objective.
        if (!inside(current, requestedAcceleration, alpha, dt, acceleration)) {
            if (!inside(current, 0, alpha, dt, acceleration)) {
                throw new IllegalStateException("No finite tank velocity transition");
            }
            // For fixed omega(t), a² + ((v0 + a*t) * omega(t))² is convex in a
            // at every t. Its feasible intersection contains zero, so bisect
            // only toward the requested acceleration, keeping the feasible end.
            // Primitive locals avoid allocating objects for search candidates.
            double feasible = 0, infeasible = 1;
            double precision = Math.min(ACCELERATION_PRECISION, 0.01 * limits.acceleration);
            for (int i = 0; i < MAX_BISECTIONS
                    && Math.abs(requestedAcceleration) * (infeasible - feasible) > precision; i++) {
                double fraction = (feasible + infeasible) / 2;
                if (inside(current, requestedAcceleration * fraction, alpha, dt, acceleration)) feasible = fraction;
                else infeasible = fraction;
            }
            forward = current.forward + requestedAcceleration * feasible * dt;
        }
        return new Result(new Velocity(forward, omega), recovering);
    }

    private static boolean inside(Velocity current, double a, double alpha, double dt, double maximum) {
        return peakAcceleration(current, a, alpha, dt) <= maximum + 1e-10;
    }

    /** Exact maximum over a linear velocity/turn-rate ramp, including its interior extremum. */
    static double peakAcceleration(Velocity current, double a, double alpha, double dt) {
        double lateral = Math.max(Math.abs(current.forward * current.omega),
            Math.abs((current.forward + a * dt) * (current.omega + alpha * dt)));
        double quadratic = a * alpha;
        if (Math.abs(quadratic) > 1e-15) {
            double t = -(a * current.omega + alpha * current.forward) / (2 * quadratic);
            if (t > 0 && t < dt) lateral = Math.max(lateral,
                Math.abs((current.forward + a * t) * (current.omega + alpha * t)));
        }
        return Math.hypot(a, lateral);
    }
}
