package org.team100.lib.profile.timed;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class JerkLimitedProfile100Test {
    private static final boolean DEBUG = false;

    @Test
    void testSimple() {
        JerkLimitedProfile100 p = new JerkLimitedProfile100(2, 6, 25, false);
        p.init(new Control100(), new Model100(1, 0));
        for (double t = 0; t < 2; t += 0.01) {
            Control100 c = p.sample(t);
            if (DEBUG)
                Util.printf("%12.3f %12.3f %12.3f %12.3f\n", t, c.x(), c.v(), c.a());
        }

    }
}
