package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

public class SwerveDeadbandTest {
    private static final double kDelta = 0.001;

    @Test
    void testLargeInput() {
        // large input should be unaffected.
        FieldRelativeVelocity input = new FieldRelativeVelocity(1, 0, 0);
        SwerveDeadband deadband = new SwerveDeadband();
        FieldRelativeVelocity result = deadband.apply(input);
        assertEquals(1, result.x(), kDelta);
        assertEquals(0, result.y(), kDelta);
        assertEquals(0, result.theta(), kDelta);
    }

    @Test
    void testSmallInput() {
        // small input should be ignored.
        // 5 mm/s is quite slow, maybe too slow?
        FieldRelativeVelocity input = new FieldRelativeVelocity(0.005, 0, 0);
        SwerveDeadband deadband = new SwerveDeadband();
        FieldRelativeVelocity result = deadband.apply(input);
        assertEquals(0, result.x(), kDelta);
        assertEquals(0, result.y(), kDelta);
        assertEquals(0, result.theta(), kDelta);
    }
}
