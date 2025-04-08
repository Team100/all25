package org.team100.lib.profile;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

/**
 * See
 * https://docs.google.com/spreadsheets/d/1oTEZ7a-SG7z21KIk3MG2os2tp4eIznZEGxuppt3LA1A
 */
public class DualProfileTest {
    private static final boolean DEBUG = false;

    void dump(double t, Model100 setpoint) {
        if (DEBUG)
            Util.printf("%12.6f %12.6f %12.6f\n", t, setpoint.x(), setpoint.v());
    }

    /**
     * This example shows the use of DualProfile for the sort of "settling" profile
     * mentioned here https://www.analog.com/en/resources/app-notes/an-013.html -- a
     * normal trapezoid or triangle, followed by a nonzero velocity until the
     * setpoint is achieved. The reason to use this method is that relying on
     * proportional feedback alone for the final positioning is slow; if we aim
     * exactly for the target, we might be ahead of it, or might be behind, and
     * since we're close, the proportional output available is small. Instead, if we
     * use very high deceleration but stop just short of the target, then we know we
     * can proceed *forward* to get there, faster than pure proportional control.
     */
    @Test
    void testVMin() {
        // baseline profile
        TrapezoidProfile100 fast = new TrapezoidProfile100(1, 1, 0.01);
        // slow one has same accel, lower speed.
        TrapezoidProfile100 slow = new TrapezoidProfile100(0.1, 1, 0.01);
        double neighborhood = 0.1;
        DualProfile profile = new DualProfile(fast, slow, neighborhood);
        Model100 setpoint = new Model100();
        final Model100 goal = new Model100(1, 0);
        double t = 0;
        // setpoint for 0
        dump(t, setpoint);
        for (int i = 0; i < 150; ++i) {
            // setpoint for +0.02s in the future
            setpoint = profile.calculate(0.02, setpoint, goal).model();
            t += 0.02;
            dump(t, setpoint);
        }
    }


}
