package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

public class SlowStartTest {
    private static final double kDelta = 0.001;

    @Test
    void testUnconstrained() {
        SlowStart c = new SlowStart();
        FieldRelativeVelocity s = c.limit(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(0, 0, 0));
        assertEquals(0, s.x(), kDelta);
        assertEquals(0, s.y(), kDelta);
        assertEquals(0, s.theta(), kDelta);
    }

    @Test
    void testConstrained() {
        SlowStart c = new SlowStart();
        FieldRelativeVelocity s = c.limit(
                new FieldRelativeVelocity(0, 0, 0),
                new FieldRelativeVelocity(1, 0, 0));
        assertEquals(0.01, s.x(), kDelta);
        assertEquals(0, s.y(), kDelta);
        assertEquals(0, s.theta(), kDelta);
    }
}
