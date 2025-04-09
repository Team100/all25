package org.team100.lib.profile.timed;

import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class CompleteProfileTest {
    private static final boolean DEBUG = true;
    private static final double DT = 0.02;

    /** Dump the sliding mode surface */
    @Test
    void testMode() {
        CompleteProfile p = new CompleteProfile(2, 6, 0.01);
        if (DEBUG) {
            for (double x = -10; x < 10; x += 0.01) {
                Control100 sample = p.m_byDistance.get(x);
                Util.printf("%12.3f %12.3f %12.3f\n", sample.x(), sample.v(), sample.a());
            }
        }
    }

    @Test
    void testSimple() {
        CompleteProfile p = new CompleteProfile(2, 6, 0.01);
        final Model100 goal = new Model100(1, 0);
        Control100 c = new Control100();
        double t = 0;
        for (int i = 0; i < 300; ++i) {
            if (DEBUG)
                Util.printf("%12.3f %12.3f %12.3f %12.3f\n", t, c.x(), c.v(), c.a());
            c = p.calculate(DT, c.model(), goal);
            t += DT;
        }
    }

    /** Moving goals are not allowed. */
    @Test
    void testMovingGoal() {
        CompleteProfile p = new CompleteProfile(2, 6, 0.01);
        assertThrows(IllegalArgumentException.class,
                () -> p.calculate(0.02, new Model100(), new Model100(1, 1)));
    }

    @Test
    void testKeyInterpolation() {

    }
}
