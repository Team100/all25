package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

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

}
