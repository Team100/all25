package org.team100.lib.profile.incremental;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class CurrentLimitedExponentialProfileTest {

    private static final boolean DEBUG = false;

    /**
     * Just to see what it looks like.
     */
    @Test
    void testRun() {
        double maxVel = 2;
        double limitedAccel = 5;
        // stall torque is *much* more than the limit :-)
        double stallAccel = 10;
        CurrentLimitedExponentialProfile profile = new CurrentLimitedExponentialProfile(
                maxVel,
                limitedAccel,
                stallAccel);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(3, 0);
        double tt = 0;
        for (int i = 0; i < 150; ++i) {
            tt += 0.02;
            sample = profile.calculate(0.02, sample, end);
            if (DEBUG)
                Util.printf("%5.3f %5.3f %5.3f\n", tt, sample.x(), sample.v());
        }
    }

    @Test
    void testAccel() {
        // motionless, doesn't really matter.
        assertFalse(
                CurrentLimitedExponentialProfile.isAccel(
                        new Control100(),
                        new Control100()));
        // initial velocity is positive, control is more positive -> accelerating
        assertTrue(
                CurrentLimitedExponentialProfile.isAccel(
                        new Control100(0, 1),
                        new Control100(0, 1.1, 1)));
        // initial velocity is positive, control is less positive -> braking
        assertFalse(
                CurrentLimitedExponentialProfile.isAccel(
                        new Control100(0, 1),
                        new Control100(0, 0.9, -1)));
        // initial velocity is positive, control is negative -> braking
        assertFalse(
                CurrentLimitedExponentialProfile.isAccel(
                        new Control100(0, 1),
                        new Control100(0, -0.9, -1)));
        // initial velocity is positive, control is negative -> braking
        assertFalse(
                CurrentLimitedExponentialProfile.isAccel(
                        new Control100(0, 1),
                        new Control100(0, -1.1, -1)));

    }

}
