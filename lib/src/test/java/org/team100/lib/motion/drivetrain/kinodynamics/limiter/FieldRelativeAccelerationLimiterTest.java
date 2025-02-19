package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

public class FieldRelativeAccelerationLimiterTest {
    private static final double kDelta = 0.001;

    @Test
    void testUnconstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter c = new FieldRelativeAccelerationLimiter(l);
        FieldRelativeVelocity s = c.limit(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(0, 0, 0));
        assertEquals(0, s.x(), kDelta);
        assertEquals(0, s.y(), kDelta);
        assertEquals(0, s.theta(), kDelta);
    }

    @Test
    void testConstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter c = new FieldRelativeAccelerationLimiter(l);
        FieldRelativeVelocity s = c.limit(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(1, 0, 0));
        assertEquals(0.02, s.x(), kDelta);
        assertEquals(0, s.y(), kDelta);
        assertEquals(0, s.theta(), kDelta);
    }

}
