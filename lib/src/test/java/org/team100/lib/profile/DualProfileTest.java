package org.team100.lib.profile;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class DualProfileTest {
    private static final boolean DEBUG = false;

    @Test
    void testSimple() {
        TrapezoidProfile100 fast = new TrapezoidProfile100(1, 1, 0.01);
        TrapezoidProfile100 slow = new TrapezoidProfile100(0.1, 1, 0.01);
        double neighborhood = 0.1;
        DualProfile profile = new DualProfile(fast, slow, neighborhood);
        Model100 setpoint = new Model100();
        Model100 goal = new Model100(1, 0);
        if (DEBUG)
            Util.printf("%s\n", setpoint);
        for (int i = 0; i < 150; ++i) {
            setpoint = profile.calculate(0.02, setpoint, goal).model();
            if (DEBUG)
                Util.printf("%s\n", setpoint);
        }
        goal = new Model100(0, 0);
        for (int i = 0; i < 150; ++i) {
            setpoint = profile.calculate(0.02, setpoint, goal).model();
            if (DEBUG)
                Util.printf("%s\n", setpoint);
        }
        
    }

}
