package org.team100.lib.profile.incremental;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Debug;

public class ExponentialProfileWPITest implements Debug {

    /**
     * Just to see what it looks like.
     */
    @Test
    void testRun() {
        double maxVel = 2;
        double maxAccel = 10;
        ExponentialProfileWPI profile = new ExponentialProfileWPI(maxVel, maxAccel);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(3, 0);
        double tt = 0;
        for (int i = 0; i < 150; ++i) {
            tt += 0.02;
            sample = profile.calculate(0.02, sample, end);
            debug("%5.3f %5.3f %5.3f\n", tt, sample.x(), sample.v());
        }
    }

    @Override
    public boolean debug() {
        return false;
    }

}
