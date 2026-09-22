package frc.robot.lib.BLine.following;

import frc.robot.lib.BLine.path.DriveDirection;
import java.util.Random;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class TankRateLimiterTest {
    @Test
    void boundsTheWholeStepWhileSteeringAndBrakingInEitherDirection() {
        Random random = new Random(2027);
        for (int trial = 0; trial < 1000; trial++) {
            double budget = random.nextDouble(0.01, 10);
            var limits = new TankRateLimiter.Limits(budget, 4, 3.5, 2.5);
            double v = random.nextDouble(-3.5, 3.5);
            double maxW = Math.min(2.5, budget / Math.max(1e-9, Math.abs(v)));
            double w = random.nextDouble(-maxW, maxW);
            var current = new TankRateLimiter.Velocity(v, w);
            var request = new TankRateLimiter.Velocity(random.nextDouble(-5, 5), random.nextDouble(-8, 8));
            var direction = trial % 2 == 0 ? DriveDirection.FORWARD : DriveDirection.BACKWARD;
            double dt = trial % 3 == 0 ? 0.005 : trial % 3 == 1 ? 0.02 : 0.04;
            var result = TankRateLimiter.limit(current, request, limits, dt, direction);
            assertFalse(result.recovering());
            double nextV = result.velocity().forward(), nextW = result.velocity().omega();
            assertTrue(Math.abs(nextV) <= 3.5 + 1e-8);
            assertTrue(Math.abs(nextW) <= 2.5 + 1e-8);
            assertTrue(Math.abs(nextW - w) <= 4 * dt + 1e-8);
            if (direction == DriveDirection.FORWARD) assertTrue(nextV >= Math.min(0, v) - 1e-8);
            else assertTrue(nextV <= Math.max(0, v) + 1e-8);
            double acceleration = (nextV - v) / dt;
            // Independently sample the actual velocity ramps, rather than assert
            // against the production peak/interval implementation itself.
            for (int sample = 0; sample <= 100; sample++) {
                double fraction = sample / 100.0;
                double atV = v + (nextV - v) * fraction;
                double atW = w + (nextW - w) * fraction;
                assertTrue(Math.hypot(acceleration, atV * atW) <= budget + 1e-7, "Full-step acceleration budget");
            }
        }
    }

    @Test
    void approachesTheAccelerationBoundaryFromBelowWithinUsefulPrecision() {
        // Almost constant turn rate with increasing speed: the maximum occurs
        // at the end of the step. Solve that quadratic independently, so an
        // over-conservative limiter (including always holding speed) fails too.
        double dt = 0.02;
        for (double budget : new double[] {0.05, 0.5, 2, 10}) {
            for (DriveDirection direction : DriveDirection.values()) {
                double sign = direction == DriveDirection.FORWARD ? 1 : -1;
                double speed = Math.sqrt(budget), omega = speed / 2;
                var current = new TankRateLimiter.Velocity(sign * speed, omega);
                var limits = new TankRateLimiter.Limits(budget, 1e-8, 100, 100);
                var result = TankRateLimiter.limit(current, new TankRateLimiter.Velocity(sign * 100, omega),
                    limits, dt, direction).velocity();
                double w = result.omega();
                double qa = 1 + w * w * dt * dt;
                double qb = 2 * w * w * speed * dt;
                double qc = w * w * speed * speed - budget * budget;
                double optimum = (-qb + Math.sqrt(qb * qb - 4 * qa * qc)) / (2 * qa);
                double actual = sign * (result.forward() - current.forward()) / dt;
                assertTrue(actual <= optimum + 1e-8, "Stay on the feasible side of the boundary");
                assertTrue(optimum - actual <= Math.min(0.02, budget * 0.01) + 1e-8,
                    "Boundary uncertainty must respect both absolute and relative precision");
            }
        }
    }

    @Test
    void keepsTheUnmodifiedRequestAtASaturatedCornerRatherThanAddingABrakingPolicy() {
        for (DriveDirection direction : DriveDirection.values()) {
            double sign = direction == DriveDirection.FORWARD ? 1 : -1;
            var current = new TankRateLimiter.Velocity(sign * 2, 1);
            var request = new TankRateLimiter.Velocity(sign * 2, 2);
            var limits = new TankRateLimiter.Limits(2, 4, 3.5, 2.5);
            var first = TankRateLimiter.limit(current, request, limits, 0.02, direction).velocity();
            assertEquals(current.forward(), first.forward(), 1e-8, "No braking budget at the initial boundary");
            // Every lower omega makes the next vector farther from the unchanged
            // request. This documents the local objective's saturation limitation.
            assertEquals(current.omega(), first.omega(), 1e-8);
        }
    }

    @Test
    void tradesSomeTurnErrorForABetterJointVelocityVectorInBothDirections() {
        for (double sign : new double[] {-1, 1}) {
            var current = new TankRateLimiter.Velocity(sign, 0.9);
            var request = new TankRateLimiter.Velocity(sign * 2, 1);
            var limits = new TankRateLimiter.Limits(2, 4, 3.5, 2.5);
            var next = TankRateLimiter.limit(current, request, limits, 0.02,
                sign > 0 ? DriveDirection.FORWARD : DriveDirection.BACKWARD).velocity();
            // Independent endpoint quadratic for the turn-priority choice 0.98.
            // Speed and omega both grow, so its peak is at the end of the step.
            double w = 0.98, qa = 1 + w * w * 0.02 * 0.02;
            double a = (-w * w * 0.02 + Math.sqrt(4 * qa - w * w)) / qa;
            var turnFirst = new TankRateLimiter.Velocity(sign * (1 + a * 0.02), w);
            assertTrue(Math.abs(next.omega() - 1) > Math.abs(w - 1));
            assertTrue(vectorError(current, next, sign * 1.04, 1, 0.02)
                < 0.8 * vectorError(current, turnFirst, sign * 1.04, 1, 0.02));
        }
    }

    @Test
    void acceptsFeasibleRequestsWithoutCoupledPreprocessingIncludingNearZeroMotion() {
        var limits = new TankRateLimiter.Limits(2, 4, 3.5, 2.5);
        for (double sign : new double[] {-1, 1}) {
            for (double speed : new double[] {1, 1e-10}) {
                var current = new TankRateLimiter.Velocity(sign * speed, 0.9);
                var request = new TankRateLimiter.Velocity(sign * speed, 0.95);
                assertEquals(request, TankRateLimiter.limit(current, request, limits, 0.02,
                    sign > 0 ? DriveDirection.FORWARD : DriveDirection.BACKWARD).velocity());
            }
        }
    }

    private static double vectorError(TankRateLimiter.Velocity initial, TankRateLimiter.Velocity next,
            double targetSpeed, double targetOmega, double dt) {
        double targetAngle = (initial.omega() + targetOmega) * dt / 2;
        double angle = (initial.omega() + next.omega()) * dt / 2;
        return Math.pow(next.forward() * Math.cos(angle) - targetSpeed * Math.cos(targetAngle), 2)
            + Math.pow(next.forward() * Math.sin(angle) - targetSpeed * Math.sin(targetAngle), 2);
    }

    @Test
    void jointScoreApproachesAnIndependentDenseVelocityGrid() {
        // A deliberately simple 2D reference: no production interval solver or
        // projection helper. Includes acceleration, braking, reverse and low speed.
        for (double[] input : new double[][] {
                {1, 0.9, 2, 1}, {1.5, 1, 0, 2}, {-0.7, -1, -3, 2}, {0.01, 0.5, 2, -2},
                // Braking while omega grows can peak inside the step. Exercise
                // the approximate boundary search, including its reverse image.
                {0.4735901723, 0.2965434234, 0, 1.4188258514, 8.8176341472, 17.5766770437},
                {-0.4735901723, 0.2965434234, 0, 1.4188258514, 8.8176341472, 17.5766770437}}) {
            double v = input[0], w = input[1], dt = 0.02;
            double budget = input.length > 4 ? input[4] : 2;
            double angularAcceleration = input.length > 4 ? input[5] : 4;
            var current = new TankRateLimiter.Velocity(v, w);
            var request = new TankRateLimiter.Velocity(input[2], input[3]);
            var limits = new TankRateLimiter.Limits(budget, angularAcceleration, 3.5, 2.5);
            var direction = v < 0 ? DriveDirection.BACKWARD : DriveDirection.FORWARD;
            var actual = TankRateLimiter.limit(current, request, limits, dt, direction).velocity();
            double target = v + Math.clamp(input[2] - v, -budget * dt, budget * dt);
            double reference = Double.POSITIVE_INFINITY;
            for (int vi = 0; vi <= 320; vi++) {
                double a = budget * (2.0 * vi / 320 - 1), nextV = v + a * dt;
                if (v < 0 ? nextV > 0 : nextV < 0) continue;
                for (int wi = 0; wi <= 320; wi++) {
                    double nextW = w + angularAcceleration * dt * (2.0 * wi / 320 - 1);
                    double error = vectorError(current, new TankRateLimiter.Velocity(nextV, nextW), target, input[3], dt);
                    if (error >= reference) continue;
                    boolean feasible = true;
                    for (int sample = 0; sample <= 100; sample++) {
                        double f = sample / 100.0;
                        if (Math.hypot(a, (v + a * dt * f) * (w + (nextW - w) * f)) > budget) {
                            feasible = false;
                            break;
                        }
                    }
                    if (feasible) reference = error;
                }
            }
            assertTrue(Double.isFinite(reference));
            assertTrue(Math.sqrt(vectorError(current, actual, target, input[3], dt))
                <= Math.sqrt(reference) + (input.length > 4 ? 0.005 : 0.00005),
                "Joint vector error versus dense reference");
        }
    }

    @Test
    void recoversLoweredLimitsWithoutInstantlyClippingExistingMotion() {
        var current = new TankRateLimiter.Velocity(4, 1);
        var limits = new TankRateLimiter.Limits(2, 3, 3, 0.8);
        for (int step = 0; step < 200; step++) {
            var result = TankRateLimiter.limit(current, new TankRateLimiter.Velocity(0, 0), limits, 0.02, DriveDirection.FORWARD);
            assertTrue(Math.abs(result.velocity().forward() - current.forward()) <= Math.max(2, Math.abs(current.forward() * current.omega())) * 0.02 + 1e-8);
            assertTrue(Math.abs(result.velocity().omega() - current.omega()) <= 0.06 + 1e-8);
            assertTrue(Math.abs(result.velocity().forward()) <= Math.abs(current.forward()) + 1e-8);
            assertTrue(Math.abs(result.velocity().omega()) <= Math.abs(current.omega()) + 1e-8);
            current = result.velocity();
        }
        assertEquals(0, current.forward(), 1e-8);
        assertEquals(0, current.omega(), 1e-8);
    }

    @Test
    void turningInPlaceUsesAngularSlewAndZeroTimeHoldsThePreviousCommand() {
        var limits = new TankRateLimiter.Limits(2, 4, 3.5, 2.5);
        var stopped = new TankRateLimiter.Velocity(0, 0);
        var requested = new TankRateLimiter.Velocity(0, 2);
        assertEquals(stopped, TankRateLimiter.limit(stopped, requested, limits, 0, DriveDirection.FORWARD).velocity());
        var step = TankRateLimiter.limit(stopped, requested, limits, 0.02, DriveDirection.FORWARD).velocity();
        assertEquals(0, step.forward(), 1e-9);
        assertEquals(0.08, step.omega(), 1e-9);
    }
}
