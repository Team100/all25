package org.team100.lib.profile.roadrunner;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

public class TrapezoidProfileGeneratorTest {
    private static final boolean DEBUG = true;
    private static final double kDelta = 0.001;

    @Test
    void testUnlimitedJerk() {
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionProfile p = TrapezoidProfileGenerator.unlimitedJerk(start, 2, 6);
        // end-state accel is max (because the transition to cruise can be sharp in the
        // non-jerk-limited case.
        assertEquals(6.000, p.get(p.duration()).a(), 0.001);
        for (double t = 0; t < p.duration(); t += 0.01) {
            MotionState state = p.get(t);
            if (DEBUG) {
                double x = state.x();
                double v = state.v();
                double a = state.a();
                double j = state.j();
                Util.printf("%8.3f %8.3f %8.3f %8.3f %8.3f\n",
                        t, x, v, a, j);
            }
        }
    }
}
