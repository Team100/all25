package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

public class FieldRelativeVelocityLimiterTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testDrive() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter limiter = new FieldRelativeVelocityLimiter(logger, limit);
        assertEquals(5, k.getMaxDriveVelocityM_S(), kDelta);
        FieldRelativeVelocity s = new FieldRelativeVelocity(10, 0, 0);
        FieldRelativeVelocity i = limiter.proportional(s);
        assertEquals(5, i.x(), kDelta);
        assertEquals(0, i.y(), kDelta);
        assertEquals(0, i.theta(), kDelta);
    }

    @Test
    void testSpin() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter limiter = new FieldRelativeVelocityLimiter(logger, limit);
        assertEquals(14.142, k.getMaxAngleSpeedRad_S(), kDelta);
        FieldRelativeVelocity target = new FieldRelativeVelocity(0, 0, -20);
        FieldRelativeVelocity i = limiter.proportional(target);
        assertEquals(0, i.x(), kDelta);
        assertEquals(0, i.y(), kDelta);
        assertEquals(-14.142, i.theta(), kDelta);

    }

    @Test
    void testMotionless() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(logger, limit);
        assertEquals(14.142, k.getMaxAngleSpeedRad_S(), kDelta);
        FieldRelativeVelocity t = new FieldRelativeVelocity(0, 0, 0);
        FieldRelativeVelocity i = l.preferRotation(t);
        assertEquals(0, i.x(), kDelta);
        assertEquals(0, i.y(), kDelta);
        assertEquals(0, i.theta(), kDelta);

    }

    @Test
    void testPreferRotation2() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(logger, limit);
        {
            // inside the envelope => no change
            FieldRelativeVelocity target = new FieldRelativeVelocity(1, 0, 1);
            FieldRelativeVelocity i = l.preferRotation(target);
            assertEquals(1, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(1, i.theta(), kDelta);
        }
        {
            // full v, half omega => half v
            FieldRelativeVelocity target = new FieldRelativeVelocity(5, 0, 7.05);
            FieldRelativeVelocity i = l.preferRotation(target);
            assertEquals(2.507, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(7.05, i.theta(), kDelta);
        }
        {
            // full v, full omega => zero v, sorry
            FieldRelativeVelocity target = new FieldRelativeVelocity(5, 0, 14.142);
            FieldRelativeVelocity i = l.preferRotation(target);
            assertEquals(0, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(14.142, i.theta(), kDelta);
        }
    }

    /** shouldn't allow any movement at 6v. */
    @Test
    void testBrownout() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 6);
        FieldRelativeVelocityLimiter limiter = new FieldRelativeVelocityLimiter(logger, limit);
        FieldRelativeVelocity target = new FieldRelativeVelocity(1, 0, 0);
        FieldRelativeVelocity limited = limiter.apply(target);
        assertEquals(0, limited.x(), kDelta);
        assertEquals(0, limited.y(), kDelta);
        assertEquals(0, limited.theta(), kDelta);
    }

    /** desaturation should keep the same instantaneous course. */
    @Test
    void testDesaturationCourseInvariant() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(logger, limit);

        { // both motionless
            FieldRelativeVelocity target = new FieldRelativeVelocity(0, 0, 0);
            assertTrue(target.angle().isEmpty());
            FieldRelativeVelocity desaturated = l.apply(target);
            assertTrue(desaturated.angle().isEmpty());
        }
        { // translating ahead
            FieldRelativeVelocity target = new FieldRelativeVelocity(10, 0, 0);
            assertEquals(0, target.angle().get().getRadians(), kDelta);
            assertEquals(10, target.norm(), kDelta);
            FieldRelativeVelocity desaturated = l.apply(target);
            assertEquals(0, desaturated.angle().get().getRadians(), kDelta);
            assertEquals(5, desaturated.norm(), kDelta);
        }
        { // translating ahead and spinning
            FieldRelativeVelocity target = new FieldRelativeVelocity(10, 0, 10);
            assertEquals(0, target.angle().get().getRadians(), kDelta);
            assertEquals(10, target.norm(), kDelta);
            assertEquals(10, target.theta(), kDelta);
            FieldRelativeVelocity desaturated = l.apply(target);
            assertEquals(0, desaturated.angle().get().getRadians(), kDelta);
            assertEquals(3.694, desaturated.norm(), kDelta);
            assertEquals(3.694, desaturated.theta(), kDelta);
        }
        { // translating 45 to the left and spinning
            FieldRelativeVelocity target = new FieldRelativeVelocity(10 / Math.sqrt(2), 10 / Math.sqrt(2), 10);
            assertEquals(Math.PI / 4, target.angle().get().getRadians(), kDelta);
            assertEquals(10, target.norm(), kDelta);
            assertEquals(10, target.theta(), kDelta);
            FieldRelativeVelocity desaturated = l.apply(target);
            assertEquals(Math.PI / 4, desaturated.angle().get().getRadians(), kDelta);
            assertEquals(2.612, desaturated.x(), kDelta);
            assertEquals(2.612, desaturated.y(), kDelta);
            // slightly different due to drivetrain geometry
            assertEquals(3.693, desaturated.norm(), kDelta);
            assertEquals(3.693, desaturated.theta(), kDelta);
        }
    }

    @Test
    void driveAndSpinLimited() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.limiting();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter limiter = new FieldRelativeVelocityLimiter(logger, limit);
        FieldRelativeVelocity target = new FieldRelativeVelocity(5, 0, 25);
        assertEquals(5, target.x(), kDelta);
        assertEquals(0, target.y(), kDelta);
        assertEquals(25, target.theta(), kDelta);
        // the spin is 5x the drive, as requested.
        // use the target as the previous setpoint
        FieldRelativeVelocity setpoint = limiter.apply(target);
        assertEquals(1.806, setpoint.x(), kDelta);
        assertEquals(0, setpoint.y(), kDelta);
        assertEquals(9.032, setpoint.theta(), kDelta);
    }

}
