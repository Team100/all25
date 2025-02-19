package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

public class FieldRelativeCapsizeLimiterTest {
    private static final double kDelta = 0.001;

    /**
     * initial = target => zero delta v => no constraint
     */
    @Test
    void testUnconstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        FieldRelativeCapsizeLimiter c = new FieldRelativeCapsizeLimiter(l);
        FieldRelativeVelocity result = c.limit(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(0, 0, 0));
        assertEquals(0, result.x(), kDelta);
        assertEquals(0, result.y(), kDelta);
        assertEquals(0, result.theta(), kDelta);
    }

    /**
     * initial (1,0) target (0,1) => delta v is 1.414 m/s.
     * accel is ~70, way over the limit of around 8
     * allowed deltav in 0.02 is 0.163, so the resulting speed should
     * be quite close to the initial value.
     */
    @Test
    void testConstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        assertEquals(8.166, l.getMaxCapsizeAccelM_S2(), kDelta);
        FieldRelativeCapsizeLimiter c = new FieldRelativeCapsizeLimiter(l);
        FieldRelativeVelocity result = c.limit(
                new FieldRelativeVelocity(1, 0, 0),
                new FieldRelativeVelocity(0, 1, 0));
        assertEquals(0.884, result.x(), kDelta);
        assertEquals(0.115, result.y(), kDelta);
        assertEquals(0, result.theta(), kDelta);
    }

    @Test
    void testLowCentripetal() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.lowCapsize();
        assertEquals(1.225, l.getMaxCapsizeAccelM_S2(), kDelta);
        FieldRelativeCapsizeLimiter c = new FieldRelativeCapsizeLimiter(l);
        FieldRelativeVelocity result = c.limit(
                new FieldRelativeVelocity(1, 0, 0),
                new FieldRelativeVelocity(0, 1, 0));
        assertEquals(0.982, result.x(), kDelta);
        assertEquals(0.017, result.y(), kDelta);
        assertEquals(0, result.theta(), kDelta);
    }

    @Test
    void testOverspeedCentripetal() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        assertEquals(24.5, l.getMaxCapsizeAccelM_S2(), kDelta);
        FieldRelativeCapsizeLimiter c = new FieldRelativeCapsizeLimiter(l);
        FieldRelativeVelocity result = c.limit(
                new FieldRelativeVelocity(5, 0, 0),
                new FieldRelativeVelocity(0, 5, 0));
        assertEquals(4.653, result.x(), kDelta);
        assertEquals(0.346, result.y(), kDelta);
        assertEquals(0, result.theta(), kDelta);
    }

}
