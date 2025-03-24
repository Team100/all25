package org.team100.lib.profile;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class ExponentialProfileWPITest {
    private static final boolean DEBUG = false;

    @Test
    void testRun() {
        // just to see what it looks like.
        ExponentialProfileWPI profile = new ExponentialProfileWPI(2, 5);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(3, 0);
        double tt = 0;
        for (int i = 0; i < 150; ++i) {
            tt += 0.02;
            sample = profile.calculate(0.02, sample.model(), end);
            if (DEBUG)
                Util.printf("%5.3f %5.3f %5.3f\n", tt, sample.x(), sample.v());
        }
    }

}
