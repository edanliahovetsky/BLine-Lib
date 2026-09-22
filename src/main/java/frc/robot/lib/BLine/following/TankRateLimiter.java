package frc.robot.lib.BLine.following;

import frc.robot.lib.BLine.path.DriveDirection;

/**
 * Jointly limits signed forward speed and turn rate without a wheel model.
 *
 * <p>Compare next field-velocity vectors in the frame of the current heading.
 * Each candidate respects sqrt(a² + (v omega)²) throughout its linear speed and
 * turn-rate ramps. At fixed next omega, feasible forward accelerations form a
 * convex interval; projection onto that heading gives the best speed in it.
 * A bounded angular search approximates the closest vector, without turn
 * priority or reducing the speed request to accommodate a requested turn.
 *
 * <p>This one-step objective is deliberately local: at a saturated corner it
 * may hold motion instead of temporarily steering away from the request to
 * make room for braking. Guidance, not a hidden limiter policy, owns the request.
 */
final class TankRateLimiter {
    private static final int ANGULAR_INTERVALS = 16;
    private static final int REFINEMENTS = 3;
    private static final int INTERVAL_ITERATIONS = 40;
    // Inner speed-bound uncertainty, not a score deadband or a constraint allowance.
    private static final double SPEED_PRECISION = 0.005;
    private static final double STEP_PRECISION_FRACTION = 0.05;
    private static final double FEASIBILITY_EPSILON = 1e-10;

    record Velocity(double forward, double omega) {}
    record Limits(double acceleration, double angularAcceleration, double speed, double omega) {}
    record Result(Velocity velocity, boolean recovering) {}
    private record Candidate(double forward, double omega, double error, double turnError) {}

    private TankRateLimiter() {}

    static Result limit(Velocity current, Velocity requested, Limits limits, double dt, DriveDirection direction) {
        if (!(dt > 0) || !Double.isFinite(dt)) return new Result(current, false);
        // Lowered limits cannot instantly remove motion. Allow its current excess
        // while preventing any larger speed or cornering excess.
        var reachable = new Limits(Math.max(limits.acceleration, Math.abs(current.forward * current.omega)),
            limits.angularAcceleration, Math.max(limits.speed, Math.abs(current.forward)),
            Math.max(limits.omega, Math.abs(current.omega)));
        boolean recovering = reachable.acceleration > limits.acceleration + FEASIBILITY_EPSILON
            || reachable.speed > limits.speed + FEASIBILITY_EPSILON
            || reachable.omega > limits.omega + FEASIBILITY_EPSILON;
        double requestedForward = Math.clamp(requested.forward,
            direction == DriveDirection.FORWARD ? 0 : -limits.speed,
            direction == DriveDirection.FORWARD ? limits.speed : 0);
        double requestedOmega = Math.clamp(requested.omega, -limits.omega, limits.omega);
        // Only the independent forward acceleration bound prepares the request.
        // Coupled acceleration is enforced on candidates, never by rescaling it.
        double targetForward = current.forward + Math.clamp(requestedForward - current.forward,
            -limits.acceleration * dt, limits.acceleration * dt);
        double targetAngle = (current.omega + requestedOmega) * dt / 2;
        double targetX = targetForward * Math.cos(targetAngle);
        double targetY = targetForward * Math.sin(targetAngle);
        double low = Math.max(-reachable.omega, current.omega - limits.angularAcceleration * dt);
        double high = Math.min(reachable.omega, current.omega + limits.angularAcceleration * dt);
        // A stationary pivot has no translational vector error to optimize.
        if (current.forward == 0 && targetForward == 0) {
            return new Result(new Velocity(0, Math.clamp(requestedOmega, low, high)), recovering);
        }
        if (requestedOmega >= low && requestedOmega <= high
            && inside(current, (targetForward - current.forward) / dt,
                (requestedOmega - current.omega) / dt, dt, reachable.acceleration)) {
            return new Result(new Velocity(targetForward, requestedOmega), recovering);
        }

        // Holding the initial state is feasible under the recovery envelope.
        Candidate best = score(current, current, requestedOmega, targetX, targetY, dt);
        var samples = new Candidate[ANGULAR_INTERVALS + 4];
        Candidate first = null;
        double spacing = (high - low) / ANGULAR_INTERVALS;
        for (int i = 0; i <= ANGULAR_INTERVALS + 3; i++) {
            double omega;
            if (i <= ANGULAR_INTERVALS) omega = low + spacing * i;
            else if (i == ANGULAR_INTERVALS + 1) omega = Math.clamp(requestedOmega, low, high);
            else if (i == ANGULAR_INTERVALS + 2) omega = current.omega;
            else if (low <= 0 && high >= 0) omega = 0;
            else continue;
            Candidate candidate = candidate(current, omega, requestedOmega, targetX, targetY, reachable, dt, direction);
            if (candidate != null) {
                samples[i] = candidate;
                first = first == null ? candidate : better(first, candidate);
                best = better(best, candidate);
            }
        }
        // Refine two neighborhoods, without assuming a globally unimodal angular
        // objective. At most 20 seeds + 12 refinements are evaluated.
        // Two linear scans select the same neighborhoods without sorting or a
        // growable list. Preserve joint score ordering, including exact ties.
        if (first == null) first = best;
        Candidate second = null;
        for (Candidate sample : samples) {
            if (sample != null && Math.abs(sample.omega - first.omega) > spacing) {
                second = second == null ? sample : better(second, sample);
            }
        }
        for (int neighborhood = 0; neighborhood < 2; neighborhood++) {
            Candidate seed = neighborhood == 0 ? first : second;
            if (seed == null) continue;
            Candidate local = seed;
            double step = spacing;
            for (int refinement = 0; refinement < REFINEMENTS; refinement++) {
                step /= 2;
                double centre = local.omega;
                local = better(local, candidate(current, Math.max(low, centre - step), requestedOmega,
                    targetX, targetY, reachable, dt, direction));
                local = better(local, candidate(current, Math.min(high, centre + step), requestedOmega,
                    targetX, targetY, reachable, dt, direction));
            }
            best = better(best, local);
        }
        return new Result(new Velocity(best.forward, best.omega), recovering);
    }

    private static Candidate candidate(Velocity current, double omega, double requestedOmega,
            double targetX, double targetY, Limits limits, double dt, DriveDirection direction) {
        double alpha = (omega - current.omega) / dt;
        double minSpeed = direction == DriveDirection.FORWARD ? Math.min(0, current.forward) : -limits.speed;
        double maxSpeed = direction == DriveDirection.BACKWARD ? Math.max(0, current.forward) : limits.speed;
        // At t=0 the remaining budget bounds a regardless of angular acceleration.
        double initialBudget = Math.sqrt(Math.max(0,
            limits.acceleration * limits.acceleration - Math.pow(current.forward * current.omega, 2)));
        double low = Math.max(-initialBudget, (minSpeed - current.forward) / dt);
        double high = Math.min(initialBudget, (maxSpeed - current.forward) / dt);
        // At t=dt the budget is a quadratic in forward acceleration a.
        double qa = 1 + omega * omega * dt * dt;
        double discriminant = limits.acceleration * limits.acceleration * qa
            - current.forward * current.forward * omega * omega;
        if (discriminant < 0) return null;
        double centre = -current.forward * omega * omega * dt / qa;
        double radius = Math.sqrt(discriminant) / qa;
        low = Math.max(low, centre - radius);
        high = Math.min(high, centre + radius);
        if (low > high) return null;

        double angle = (current.omega + omega) * dt / 2;
        double cosine = Math.cos(angle), sine = Math.sin(angle);
        double projectedSpeed = targetX * cosine + targetY * sine;
        double acceleration = Math.clamp((projectedSpeed - current.forward) / dt, low, high);
        if (!inside(current, acceleration, alpha, dt, limits.acceleration)) {
            // Endpoint bounds can miss an interior peak. The full-step peak is
            // convex in a. Find a feasible anchor, then the boundary toward the
            // projection. Zero acceleration need not be feasible for this omega.
            double anchor = (low + high) / 2;
            if (inside(current, low, alpha, dt, limits.acceleration)) anchor = low;
            else if (inside(current, high, alpha, dt, limits.acceleration)) anchor = high;
            else if (!inside(current, anchor, alpha, dt, limits.acceleration)) {
                double left = low, right = high;
                for (int i = 0; i < INTERVAL_ITERATIONS; i++) {
                    double x = left + (right - left) / 3, y = right - (right - left) / 3;
                    if (peakAcceleration(current, x, alpha, dt) < peakAcceleration(current, y, alpha, dt)) right = y;
                    else left = x;
                }
                anchor = (left + right) / 2;
                if (!inside(current, anchor, alpha, dt, limits.acceleration)) return null;
            }
            double feasible = anchor, infeasible = acceleration;
            double precision = Math.min(SPEED_PRECISION / dt, STEP_PRECISION_FRACTION * limits.acceleration);
            for (int i = 0; i < INTERVAL_ITERATIONS && Math.abs(feasible - infeasible) > precision; i++) {
                double midpoint = (feasible + infeasible) / 2;
                if (inside(current, midpoint, alpha, dt, limits.acceleration)) feasible = midpoint;
                else infeasible = midpoint;
            }
            acceleration = feasible;
        }
        double forward = current.forward + acceleration * dt;
        if (!inside(current, (forward - current.forward) / dt, alpha, dt, limits.acceleration)) return null;
        return score(forward, omega, requestedOmega, targetX, targetY, cosine, sine);
    }

    private static Candidate score(Velocity current, Velocity next, double requestedOmega,
            double targetX, double targetY, double dt) {
        double angle = (current.omega + next.omega) * dt / 2;
        return score(next.forward, next.omega, requestedOmega, targetX, targetY, Math.cos(angle), Math.sin(angle));
    }

    private static Candidate score(double forward, double omega, double requestedOmega,
            double targetX, double targetY, double cosine, double sine) {
        double dx = forward * cosine - targetX;
        double dy = forward * sine - targetY;
        return new Candidate(forward, omega, dx * dx + dy * dy, Math.abs(omega - requestedOmega));
    }

    private static Candidate better(Candidate left, Candidate right) {
        if (right == null) return left;
        // Only exact vector-score ties use heading preference. Near-zero motion
        // must not silently switch back to a turn-priority controller.
        return right.error < left.error || (right.error == left.error && right.turnError < left.turnError)
            ? right : left;
    }

    private static boolean inside(Velocity current, double a, double alpha, double dt, double maximum) {
        double lateral = peakLateralAcceleration(current, a, alpha, dt);
        double bound = maximum + FEASIBILITY_EPSILON;
        double squaredBound = bound * bound;
        // Comparing squared magnitudes avoids a square root at every feasibility
        // check. Retain hypot's overflow handling for unusually large inputs.
        return Double.isFinite(squaredBound)
            ? a * a + lateral * lateral <= squaredBound
            : Math.hypot(a, lateral) <= bound;
    }

    /** Exact maximum over linear speed/turn-rate ramps, including the interior extremum. */
    static double peakAcceleration(Velocity current, double a, double alpha, double dt) {
        return Math.hypot(a, peakLateralAcceleration(current, a, alpha, dt));
    }

    private static double peakLateralAcceleration(Velocity current, double a, double alpha, double dt) {
        double lateral = Math.max(Math.abs(current.forward * current.omega),
            Math.abs((current.forward + a * dt) * (current.omega + alpha * dt)));
        double quadratic = a * alpha;
        if (quadratic != 0) {
            double t = -(a * current.omega + alpha * current.forward) / (2 * quadratic);
            if (t > 0 && t < dt) lateral = Math.max(lateral,
                Math.abs((current.forward + a * t) * (current.omega + alpha * t)));
        }
        return lateral;
    }
}
