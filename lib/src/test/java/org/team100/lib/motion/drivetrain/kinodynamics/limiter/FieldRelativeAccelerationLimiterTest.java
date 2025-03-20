package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

public class FieldRelativeAccelerationLimiterTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testUnconstrained() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter c = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        FieldRelativeVelocity s = c.apply(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(0, 0, 0));
        assertEquals(0, s.x(), kDelta);
        assertEquals(0, s.y(), kDelta);
        assertEquals(0, s.theta(), kDelta);
    }

    @Test
    void testConstrained() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter c = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        FieldRelativeVelocity s = c.apply(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(1, 0, 0));
        assertEquals(0.02, s.x(), kDelta);
        assertEquals(0, s.y(), kDelta);
        assertEquals(0, s.theta(), kDelta);
    }

    @Test
    void testAlpha() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter c = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        FieldRelativeVelocity s = c.apply(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(1, 0, 1));
        assertEquals(0.02, s.x(), kDelta);
        assertEquals(0, s.y(), kDelta);
        assertEquals(0.02, s.theta(), kDelta);
    }

    @Test
    void testAlphaRatio() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter c = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        FieldRelativeVelocity s = c.apply(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(1, 0, 10));
        assertEquals(0.017, s.x(), kDelta);
        assertEquals(0, s.y(), kDelta);
        assertEquals(0.170, s.theta(), kDelta);
    }

    @Test
    void testPureAlpha() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter c = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        FieldRelativeVelocity s = c.apply(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(0, 0, 1));
        assertEquals(0, s.x(), kDelta);
        assertEquals(0, s.y(), kDelta);
        assertEquals(0.170, s.theta(), kDelta);
    }
}
