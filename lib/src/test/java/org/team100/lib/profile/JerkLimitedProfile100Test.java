package org.team100.lib.profile;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class JerkLimitedProfile100Test {
    private static final boolean DEBUG = true;

    @Test
    void testSimple() {
        JerkLimitedProfile100 p = new JerkLimitedProfile100(1, 5, 25);
        Control100 c = new Control100();
        for (int i = 0; i < 150; ++i) {
            c = p.calculate(0.02, c.model(), new Model100(1, 0));
            if (DEBUG)
                Util.printf("%5.2f %5.2f %5.2f\n", c.x(), c.v(), c.a());
        }

    }
}
