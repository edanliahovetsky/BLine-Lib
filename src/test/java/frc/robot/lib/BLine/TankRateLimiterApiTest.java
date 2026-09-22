package frc.robot.lib.BLine;

import frc.robot.lib.BLine.following.TankRateLimiter;
import frc.robot.lib.BLine.path.DriveDirection;
import org.junit.jupiter.api.Test;
import org.wpilib.math.kinematics.ChassisVelocities;
import static org.junit.jupiter.api.Assertions.*;

class TankRateLimiterApiTest {
    @Test
    void publicUtilitySupportsStraightTravelPivotsAndDirectionChangesWithoutMutatingInputs() {
        var previous = new ChassisVelocities(1, 0, 0);
        var requested = new ChassisVelocities(4, .2, 0);
        var forward = TankRateLimiter.limit(requested, previous, .02, 3, 6, 4, 5, DriveDirection.FORWARD);
        assertEquals(1.06, forward.vx, 1e-12);
        assertEquals(0, forward.vy);
        assertEquals(0, forward.omega);
        assertEquals(4, requested.vx);
        assertEquals(.2, requested.vy);
        assertEquals(1, previous.vx);

        var reverse = TankRateLimiter.limit(new ChassisVelocities(-4, 0, 0), previous,
            .02, 3, 6, 4, 5, DriveDirection.BACKWARD);
        assertEquals(.94, reverse.vx, 1e-12, "Changing direction must brake, not instantly reverse");
        var pivot = TankRateLimiter.limit(new ChassisVelocities(0, 0, 5), new ChassisVelocities(),
            .02, 3, 6, 4, 5, DriveDirection.FORWARD);
        assertEquals(0, pivot.vx);
        assertEquals(.12, pivot.omega, 1e-12);
    }

    @Test
    void publicUtilityRejectsInputsThatCannotDefineAFiniteStep() {
        var zero = new ChassisVelocities();
        assertThrows(IllegalArgumentException.class,
            () -> TankRateLimiter.limit(zero, zero, 0, 3, 6, 4, 5, DriveDirection.FORWARD));
        assertThrows(IllegalArgumentException.class,
            () -> TankRateLimiter.limit(zero, zero, .02, Double.NaN, 6, 4, 5, DriveDirection.FORWARD));
        assertThrows(IllegalArgumentException.class,
            () -> TankRateLimiter.limit(new ChassisVelocities(Double.NaN, 0, 0), zero,
                .02, 3, 6, 4, 5, DriveDirection.FORWARD));
    }
}
