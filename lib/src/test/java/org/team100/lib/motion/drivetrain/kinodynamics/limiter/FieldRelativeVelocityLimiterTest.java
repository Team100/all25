package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

public class FieldRelativeVelocityLimiterTest {
    private static final double kDelta = 0.001;

    @Test
    void testAnalyticDesaturation4() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.get();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(limit);
        {
            // verify the sign of the omega clamp
            assertEquals(14.142, k.getMaxAngleSpeedRad_S(), kDelta);
            FieldRelativeVelocity s = new FieldRelativeVelocity(0, 0, -20);
            FieldRelativeVelocity i = l.proportional(s);
            assertEquals(0, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(-14.142, i.theta(), kDelta);
        }
    }

    @Test
    void testPreferRotation() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.get();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(limit);
        assertEquals(14.142, k.getMaxAngleSpeedRad_S(), kDelta);
        {
            // trivial case works
            FieldRelativeVelocity t = new FieldRelativeVelocity(0, 0, 0);
            FieldRelativeVelocity i = l.preferRotation(t);
            assertEquals(0, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(0, i.theta(), kDelta);
        }
    }

    @Test
    void testPreferRotation2() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.get();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(limit);
        {
            // inside the envelope => no change
            FieldRelativeVelocity t = new FieldRelativeVelocity(1, 0, 1);
            FieldRelativeVelocity i = l.preferRotation(t);
            assertEquals(1, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(1, i.theta(), kDelta);
        }
        {
            // full v, half omega => half v
            FieldRelativeVelocity t = new FieldRelativeVelocity(5, 0, 7.05);
            FieldRelativeVelocity i = l.preferRotation(t);
            assertEquals(2.507, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(7.05, i.theta(), kDelta);
        }
        {
            // full v, full omega => zero v, sorry
            FieldRelativeVelocity t = new FieldRelativeVelocity(5, 0, 14.142);
            FieldRelativeVelocity i = l.preferRotation(t);
            assertEquals(0, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(14.142, i.theta(), kDelta);
        }
    }

    /** shouldn't allow any movement at 6v. */
    @Test
    void testBrownout() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.get();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(k, () -> 6);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(limit);
        FieldRelativeVelocity target = new FieldRelativeVelocity(1, 0, 0);
        FieldRelativeVelocity limited = l.limit(target);
        assertEquals(0, limited.x(), kDelta);
        assertEquals(0, limited.y(), kDelta);
        assertEquals(0, limited.theta(), kDelta);
    }

    /** desaturation should keep the same instantaneous course. */
    @Test
    void testDesaturationCourseInvariant() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.get();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(limit);

        { // both motionless
            FieldRelativeVelocity speed = new FieldRelativeVelocity(0, 0, 0);
            assertTrue(speed.angle().isEmpty());
            FieldRelativeVelocity desaturated = l.limit(speed);
            assertTrue(desaturated.angle().isEmpty());
        }
        { // translating ahead
            FieldRelativeVelocity speed = new FieldRelativeVelocity(10, 0, 0);
            assertEquals(0, speed.angle().get().getRadians(), kDelta);
            assertEquals(10, speed.norm(), kDelta);
            FieldRelativeVelocity desaturated = l.limit(speed);
            assertEquals(0, desaturated.angle().get().getRadians(), kDelta);
            assertEquals(5, desaturated.norm(), kDelta);
        }
        { // translating ahead and spinning
            FieldRelativeVelocity speed = new FieldRelativeVelocity(10, 0, 10);
            assertEquals(0, speed.angle().get().getRadians(), kDelta);
            assertEquals(10, speed.norm(), kDelta);
            assertEquals(10, speed.theta(), kDelta);
            FieldRelativeVelocity desaturated = l.limit(speed);
            assertEquals(0, desaturated.angle().get().getRadians(), kDelta);
            assertEquals(3.694, desaturated.norm(), kDelta);
            assertEquals(3.694, desaturated.theta(), kDelta);
        }
        { // translating 45 to the left and spinning
            FieldRelativeVelocity speed = new FieldRelativeVelocity(10 / Math.sqrt(2), 10 / Math.sqrt(2), 10);
            assertEquals(Math.PI / 4, speed.angle().get().getRadians(), kDelta);
            assertEquals(10, speed.norm(), kDelta);
            assertEquals(10, speed.theta(), kDelta);
            FieldRelativeVelocity desaturated = l.limit(speed);
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
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(limit);
        FieldRelativeVelocity target = new FieldRelativeVelocity(5, 0, 25);
        assertEquals(5, target.x(), kDelta);
        assertEquals(0, target.y(), kDelta);
        assertEquals(25, target.theta(), kDelta);
        // the spin is 5x the drive, as requested.
        // use the target as the previous setpoint
        FieldRelativeVelocity setpoint = l.limit(target);
        assertEquals(1.806, setpoint.x(), kDelta);
        assertEquals(0, setpoint.y(), kDelta);
        assertEquals(9.032, setpoint.theta(), kDelta);
    }

}
