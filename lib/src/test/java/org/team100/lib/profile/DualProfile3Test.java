package org.team100.lib.profile;

import org.junit.jupiter.api.Test;
import org.team100.lib.profile.timed.JerkLimitedProfile100;
import org.team100.lib.profile.timed.TimedProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class DualProfile3Test {
    private static final boolean DEBUG = false;

    void dump(double t, Control100 setpoint) {
        if (DEBUG)
            Util.printf("%12.6f %12.6f %12.6f\n", t, setpoint.x(), setpoint.v());
    }

    /** Timed dual profile, fast initially, gentle after awhile. */
    @Test
    void testFastSlow() {
        // start fast
        TimedProfile start = new JerkLimitedProfile100(1, 2, 100);
        // end slow
        TimedProfile end = new JerkLimitedProfile100(1, 0.5, 20);
        // note the time here is hand-tuned.  If it's wrong, then the
        // profile will violate its constraint (i.e. go too fast)
        DualProfile3 profile = new DualProfile3(start, end, 0.37);
        Control100 setpoint = new Control100();
        final Model100 goal = new Model100(1, 0);
        profile.init(setpoint, goal);
        double t = 0;
        // setpoint for 0
        dump(t, setpoint);
        for (int i = 0; i < 150; ++i) {
            // setpoint for +0.02s in the future
            setpoint = profile.sample(t);
            t += 0.02;
            dump(t, setpoint);
        }
    }

}
