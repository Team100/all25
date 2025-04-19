package org.team100.lib.profile.timed;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class TaperedProfileTest {
    private static final boolean DEBUG = false;

    /**
     * see
     * https://docs.google.com/spreadsheets/d/1VKc6t_AHfW9Ovo8R1cov4UfGLI1Ab3QKrIuhZbfsCR8/edit?gid=297712153#gid=297712153
     */
    @Test
    void testSimple() {
        // use a higher resolution to make a nice looking graph.
        TaperedProfile p = new TaperedProfile(2, 6, 20, 0.01);
        p.init(new Control100(), new Model100(1, 0));
        for (double t = 0; t < 2; t += 0.01) {
            Control100 c = p.sample(t);
            if (DEBUG)
                Util.printf("%12.3f %12.3f %12.3f %12.3f\n", t, c.x(), c.v(), c.a());
        }
    }

    /** make sure it also works in the opposite direction :-) */
    @Test
    void testNegative() {
        TaperedProfile p = new TaperedProfile(2, 6, 20, 0.01);
        p.init(new Control100(1, 0), new Model100());
        for (double t = 0; t < 2; t += 0.01) {
            Control100 c = p.sample(t);
            if (DEBUG)
                Util.printf("%12.3f %12.3f %12.3f %12.3f\n", t, c.x(), c.v(), c.a());
        }
    }

    /** This case works, but other cases don't work. */
    @Test
    void testMovingEntry() {
        TaperedProfile p = new TaperedProfile(2, 6, 20, 0.01);
        p.init(new Control100(1, 1), new Model100());
        for (double t = 0; t < 2; t += 0.01) {
            Control100 c = p.sample(t);
            if (DEBUG)
                Util.printf("%12.3f %12.3f %12.3f %12.3f\n", t, c.x(), c.v(), c.a());
        }
    }

    /** This doesn't work: velocity is immediately clamped to the goal path. */
    @Test
    void testOvershoot() {
        TaperedProfile p = new TaperedProfile(5, 1, 20, 0.01);
        p.init(new Control100(-1, 5), new Model100());
        for (double t = 0; t < 2; t += 0.01) {
            Control100 c = p.sample(t);
            if (DEBUG)
                Util.printf("%12.3f %12.3f %12.3f %12.3f\n", t, c.x(), c.v(), c.a());
        }
    }
}
