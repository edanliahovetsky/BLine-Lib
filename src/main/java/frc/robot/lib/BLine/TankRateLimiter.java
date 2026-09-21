package frc.robot.lib.BLine;

/**
 * Limits signed forward velocity and turn rate, without a wheel model.
 *
 * <p>With linear ramps v(t) and omega(t), the centre's field acceleration is
 * sqrt(a² + (v(t) omega(t))²). At a fixed next omega, the permissible forward
 * accelerations form an interval. The endpoints come from quadratic bounds;
 * a convex interval search includes any interior peak. A bounded angular search
 * then chooses the feasible turn rate closest to the PID request, with forward
 * speed as the tie breaker. No mixed-unit cost or trajectory is involved.
 */
final class TankRateLimiter {
    private static final double EPSILON = 1e-9;
    private static final int SAMPLES = 32;
    private static final int BISECTIONS = 40;

    record Velocity(double forward, double omega) {}
    record Limits(double acceleration, double angularAcceleration, double speed, double omega) {}
    record Result(Velocity velocity, boolean recovering) {}
    private record Interval(double low, double high) {}
    private record Candidate(double forward, double omega, double turnError, double speedError) {}

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
        Limits reachable = new Limits(acceleration, limits.angularAcceleration, speed, turnRate);
        double low = Math.max(-turnRate, current.omega - limits.angularAcceleration * dt);
        double high = Math.min(turnRate, current.omega + limits.angularAcceleration * dt);

        Candidate best = candidate(current, targetForward, targetOmega, Math.clamp(targetOmega, low, high), reachable, dt, direction);
        best = better(best, candidate(current, targetForward, targetOmega, current.omega, reachable, dt, direction));
        if (low <= 0 && high >= 0) best = better(best, candidate(current, targetForward, targetOmega, 0, reachable, dt, direction));
        double spacing = (high - low) / SAMPLES;
        for (int i = 0; i <= SAMPLES; i++) {
            best = better(best, candidate(current, targetForward, targetOmega, low + spacing * i, reachable, dt, direction));
        }
        // Refine around the best sampled feasible point. A fixed iteration count
        // gives reproducible work independent of roboRIO or desktop runner speed.
        for (int refinement = 0; best != null && refinement < 8; refinement++) {
            double centre = best.omega;
            spacing /= 2;
            best = better(best, candidate(current, targetForward, targetOmega, Math.max(low, centre - spacing), reachable, dt, direction));
            best = better(best, candidate(current, targetForward, targetOmega, Math.min(high, centre + spacing), reachable, dt, direction));
        }
        // Holding the current state is feasible under the recovery envelope.
        // Failure therefore indicates invalid arithmetic/inputs, not permission
        // to emit an unconstrained command.
        if (best == null) throw new IllegalStateException("No finite tank velocity transition");
        return new Result(new Velocity(best.forward, best.omega), recovering);
    }

    private static Candidate candidate(Velocity current, double targetForward, double targetOmega,
            double omega, Limits limits, double dt, DriveDirection direction) {
        Interval interval = interval(current, omega, limits, dt);
        if (interval == null) return null;
        double low = current.forward + interval.low * dt;
        double high = current.forward + interval.high * dt;
        // Existing opposite-direction motion may brake; it may not grow.
        if (direction == DriveDirection.FORWARD) low = Math.max(low, Math.min(0, current.forward));
        else high = Math.min(high, Math.max(0, current.forward));
        if (low > high + EPSILON) return null;
        if (low > high) low = high = (low + high) / 2;
        double forward = Math.clamp(targetForward, low, high);
        return new Candidate(forward, omega, Math.abs(omega - targetOmega), Math.abs(forward - targetForward));
    }

    private static Candidate better(Candidate left, Candidate right) {
        if (right == null) return left;
        if (left == null || right.turnError < left.turnError - EPSILON
            || (Math.abs(right.turnError - left.turnError) <= EPSILON && right.speedError < left.speedError)) return right;
        return left;
    }

    private static Interval interval(Velocity current, double omega, Limits limits, double dt) {
        double alpha = (omega - current.omega) / dt;
        double low = Math.max(-limits.acceleration, (-limits.speed - current.forward) / dt);
        double high = Math.min(limits.acceleration, (limits.speed - current.forward) / dt);
        // Acceleration budget at t=0 and t=dt. Each yields a quadratic in a.
        for (double t : new double[] {0, dt}) {
            double w = current.omega + alpha * t;
            double qa = 1 + w * w * t * t;
            double qb = 2 * w * w * current.forward * t;
            double qc = Math.pow(w * current.forward, 2) - limits.acceleration * limits.acceleration;
            double discriminant = qb * qb - 4 * qa * qc;
            if (discriminant < -EPSILON) return null;
            double root = Math.sqrt(Math.max(0, discriminant));
            low = Math.max(low, (-qb - root) / (2 * qa));
            high = Math.min(high, (-qb + root) / (2 * qa));
        }
        if (low > high + EPSILON) return null;
        if (low > high) low = high = (low + high) / 2;
        double anchor = (low + high) / 2;
        if (inside(current, low, alpha, dt, limits.acceleration)) anchor = low;
        else if (inside(current, high, alpha, dt, limits.acceleration)) anchor = high;
        else if (!inside(current, anchor, alpha, dt, limits.acceleration)) {
            double l = low, h = high;
            for (int i = 0; i < BISECTIONS; i++) {
                double x = l + (h - l) / 3, y = h - (h - l) / 3;
                if (peakAcceleration(current, x, alpha, dt) < peakAcceleration(current, y, alpha, dt)) h = y;
                else l = x;
            }
            anchor = (l + h) / 2;
            if (!inside(current, anchor, alpha, dt, limits.acceleration)) return null;
        }
        if (!inside(current, low, alpha, dt, limits.acceleration)) {
            double l = low, h = anchor;
            for (int i = 0; i < BISECTIONS; i++) {
                double mid = (l + h) / 2;
                if (inside(current, mid, alpha, dt, limits.acceleration)) h = mid;
                else l = mid;
            }
            low = h;
        }
        if (!inside(current, high, alpha, dt, limits.acceleration)) {
            double l = anchor, h = high;
            for (int i = 0; i < BISECTIONS; i++) {
                double mid = (l + h) / 2;
                if (inside(current, mid, alpha, dt, limits.acceleration)) l = mid;
                else h = mid;
            }
            high = l;
        }
        return new Interval(low, high);
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
