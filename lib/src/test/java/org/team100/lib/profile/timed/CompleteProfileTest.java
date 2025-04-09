package org.team100.lib.profile.timed;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class CompleteProfileTest {
    private static final boolean DEBUG = true;

    @Test
    void testSimple() {
        // use a higher resolution to make a nice looking graph.
        // CompleteProfile p = new CompleteProfile(2, 6, 20, 0.01);
        // p.init(new Control100(), new Model100(1, 0));
        // for (double t = 0; t < 2; t += 0.01) {
        //     Control100 c = p.sample(t);
        //     if (DEBUG)
        //         Util.printf("%12.3f %12.3f %12.3f %12.3f\n", t, c.x(), c.v(), c.a());
        // }
    }

    @Test
    void testKeyInterpolation() {
        
    }
}
