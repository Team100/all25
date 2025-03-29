package org.team100.lib.profile.timed;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class SepticSplineProfileTest {
    private static final boolean DEBUG = false;

    @Test
    void testSimple() {
        SepticSplineProfile p = new SepticSplineProfile(2, 6);
        p.init(new Model100(), new Model100(1, 0));
        for (double t = 0; t < 2; t += 0.01) {
            Control100 c = p.sample(t);
            if (DEBUG)
                Util.printf("%12.3f %12.3f %12.3f %12.3f\n", t, c.x(), c.v(), c.a());
        }
    }

    /** Start == end, zero duration */
    @Test
    void testZero() {
        SepticSplineProfile p = new SepticSplineProfile(1, 1);
        p.init(new Model100(1, 0), new Model100(1, 0));
        Control100 c = p.sample(0);
        assertEquals(1, c.x());
        assertEquals(0, c.v());
        assertEquals(0, c.a());
    }
}
