package org.team100.lib.profile;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class DualProfile2Test {
    private static final boolean DEBUG = false;

    void dump(double t, Model100 setpoint) {
        if (DEBUG)
            Util.printf("%12.6f %12.6f %12.6f\n", t, setpoint.x(), setpoint.v());
    }

    /**
     * This example shows the use of DualProfile2 for a common use case: maximum
     * effort when starting the profile, but ending more softly, to avoid
     * vibration.
     */
    @Test
    void testFastSlow() {
        // start fast
        TrapezoidProfile100 start = new TrapezoidProfile100(1, 1, 0.01);
        // end slow
        TrapezoidProfile100 end = new TrapezoidProfile100(1, 0.5, 0.01);
        DualProfile2 profile = new DualProfile2(start, end);
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
