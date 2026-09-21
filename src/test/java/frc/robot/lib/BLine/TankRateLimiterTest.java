package frc.robot.lib.BLine;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Random;
import org.junit.jupiter.api.Test;

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
    void freesCorneringCapacityBeforeBrakingIntoATighterTurn() {
        for (DriveDirection direction : DriveDirection.values()) {
            double sign = direction == DriveDirection.FORWARD ? 1 : -1;
            var current = new TankRateLimiter.Velocity(sign * 2, 1);
            var request = new TankRateLimiter.Velocity(sign * 2, 2);
            var limits = new TankRateLimiter.Limits(2, 4, 3.5, 2.5);
            var first = TankRateLimiter.limit(current, request, limits, 0.02, direction).velocity();
            assertEquals(current.forward(), first.forward(), 1e-8, "No braking budget at the initial boundary");
            assertTrue(first.omega() < current.omega(), "Free lateral capacity instead of remaining in an orbit");
            for (int step = 0; step < 300; step++) {
                current = TankRateLimiter.limit(current, request, limits, 0.02, direction).velocity();
            }
            assertEquals(sign, current.forward(), 0.02);
            assertEquals(2, current.omega(), 0.02);
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
