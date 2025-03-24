package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

public class FieldRelativeAccelerationLimiterTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testMotionless() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter limiter = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        FieldRelativeVelocity result = limiter.apply(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(0, 0, 0));
        assertEquals(0, result.x(), kDelta);
        assertEquals(0, result.y(), kDelta);
        assertEquals(0, result.theta(), kDelta);
    }

    @Test
    void testConstrained() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter limiter = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        FieldRelativeVelocity result = limiter.apply(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(1, 0, 0));
        assertEquals(0.02, result.x(), kDelta);
        assertEquals(0, result.y(), kDelta);
        assertEquals(0, result.theta(), kDelta);
    }

    @Test
    void testCartesianScale() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter limiter = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        FieldRelativeVelocity prev = new FieldRelativeVelocity(0, 0, 0);
        FieldRelativeVelocity target = new FieldRelativeVelocity(1, 0, 0);
        FieldRelativeAcceleration accel = target.accel(
                prev,
                TimedRobot100.LOOP_PERIOD_S);
        assertEquals(50, accel.x(), kDelta);
        double scale = limiter.cartesianScale(prev, target, accel);
        // allowed accel is 1, desired is 50, so scale is 0.02.
        assertEquals(0.02, scale, kDelta);
    }

    @Test
    void testAlpha() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter limiter = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        FieldRelativeVelocity result = limiter.apply(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(1, 0, 1));
        assertEquals(0.02, result.x(), kDelta);
        assertEquals(0, result.y(), kDelta);
        assertEquals(0.02, result.theta(), kDelta);
    }

    @Test
    void testAlphaRatio() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter limiter = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        FieldRelativeVelocity result = limiter.apply(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(1, 0, 10));
        assertEquals(0.017, result.x(), kDelta);
        assertEquals(0, result.y(), kDelta);
        assertEquals(0.170, result.theta(), kDelta);
    }

    @Test
    void testPureAlpha() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter limiter = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        FieldRelativeVelocity result = limiter.apply(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(0, 0, 1));
        assertEquals(0, result.x(), kDelta);
        assertEquals(0, result.y(), kDelta);
        assertEquals(0.170, result.theta(), kDelta);
    }
}
