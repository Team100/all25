package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

public class FieldRelativeCapsizeLimiterTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    /**
     * initial = target => zero delta v => no constraint
     */
    @Test
    void testUnconstrained() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeCapsizeLimiter c = new FieldRelativeCapsizeLimiter(logger, limits);
        FieldRelativeVelocity result = c.apply(
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
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        assertEquals(8.166, limits.getMaxCapsizeAccelM_S2(), kDelta);
        FieldRelativeCapsizeLimiter c = new FieldRelativeCapsizeLimiter(logger, limits);
        FieldRelativeVelocity result = c.apply(
                new FieldRelativeVelocity(1, 0, 0),
                new FieldRelativeVelocity(0, 1, 0));
        assertEquals(0.884, result.x(), kDelta);
        assertEquals(0.115, result.y(), kDelta);
        assertEquals(0, result.theta(), kDelta);
    }

    @Test
    void testLowCentripetal() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.lowCapsize();
        assertEquals(1.225, limits.getMaxCapsizeAccelM_S2(), kDelta);
        FieldRelativeCapsizeLimiter c = new FieldRelativeCapsizeLimiter(logger, limits);
        FieldRelativeVelocity result = c.apply(
                new FieldRelativeVelocity(1, 0, 0),
                new FieldRelativeVelocity(0, 1, 0));
        assertEquals(0.982, result.x(), kDelta);
        assertEquals(0.017, result.y(), kDelta);
        assertEquals(0, result.theta(), kDelta);
    }

    @Test
    void testOverspeedCentripetal() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest();
        assertEquals(8.166, limits.getMaxCapsizeAccelM_S2(), kDelta);
        FieldRelativeCapsizeLimiter c = new FieldRelativeCapsizeLimiter(logger, limits);
        FieldRelativeVelocity result = c.apply(
                new FieldRelativeVelocity(5, 0, 0),
                new FieldRelativeVelocity(0, 5, 0));
        assertEquals(4.884, result.x(), kDelta);
        assertEquals(0.115, result.y(), kDelta);
        assertEquals(0, result.theta(), kDelta);
    }

}
