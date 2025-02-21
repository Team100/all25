package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeJerk;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

public class JerkLimiterTest {
    private static final double kDelta = 0.001;

    @Test
    void testHigh() {
        // verify jerk calculation
        FieldRelativeVelocity v0 = new FieldRelativeVelocity(0, 0, 0);
        FieldRelativeVelocity v1 = new FieldRelativeVelocity(0.02, 0, 0);
        FieldRelativeVelocity v2 = new FieldRelativeVelocity(0.06, 0, 0);

        FieldRelativeAcceleration a0 = v1.accel(v0, 0.02);
        FieldRelativeAcceleration a1 = v2.accel(v1, 0.02);

        assertEquals(1, a0.x(), kDelta);
        assertEquals(2, a1.x(), kDelta);

        FieldRelativeJerk j0 = a1.jerk(a0, 0.02);
        assertEquals(50, j0.x(), kDelta);
    }

    @Test
    void testZero() {
        // verify jerk calculation
        FieldRelativeVelocity v0 = new FieldRelativeVelocity(0, 0, 0);
        FieldRelativeVelocity v1 = new FieldRelativeVelocity(0.02, 0, 0);
        FieldRelativeVelocity v2 = new FieldRelativeVelocity(0.04, 0, 0);

        FieldRelativeAcceleration a0 = v1.accel(v0, 0.02);
        FieldRelativeAcceleration a1 = v2.accel(v1, 0.02);

        assertEquals(1, a0.x(), kDelta);
        assertEquals(1, a1.x(), kDelta);

        FieldRelativeJerk j0 = a1.jerk(a0, 0.02);
        assertEquals(0, j0.x(), kDelta);
    }

    @Test
    void testUnlimited() {
        // input below the jerk limit is allowed verbatim
        FieldRelativeVelocity v0 = new FieldRelativeVelocity(0, 0, 0);
        FieldRelativeVelocity v1 = new FieldRelativeVelocity(0.02, 0, 0);
        FieldRelativeVelocity v2 = new FieldRelativeVelocity(0.04, 0, 0);
        JerkLimiter limiter = new JerkLimiter();
        FieldRelativeVelocity result = limiter.apply(v0, v1, v2);
        assertEquals(0.04, result.x(), kDelta);
    }

    @Test
    void testLimited() {
        // input above the jerk limit results in reduced velocity
        FieldRelativeVelocity v0 = new FieldRelativeVelocity(0, 0, 0);
        FieldRelativeVelocity v1 = new FieldRelativeVelocity(0.02, 0, 0);
        FieldRelativeVelocity v2 = new FieldRelativeVelocity(0.06, 0, 0);
        JerkLimiter limiter = new JerkLimiter();
        FieldRelativeVelocity result = limiter.apply(v0, v1, v2);
        assertEquals(0.05, result.x(), kDelta);
    }
}
