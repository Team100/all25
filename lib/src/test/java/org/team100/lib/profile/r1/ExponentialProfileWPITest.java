package org.team100.lib.profile.r1;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Takt;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

public class ExponentialProfileWPITest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

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
        @SuppressWarnings("unused")
        double tt = 0;
        for (int i = 0; i < 150; ++i) {
            tt += 0.02;
            sample = profile.calculate(0.02, sample, end);
            if (DEBUG)
                System.out.printf("%5.3f %5.3f %5.3f\n", tt, sample.x(), sample.v());
        }
    }

    @Test
    void testSolve() {
        double maxVel = 2;
        double maxAccel = 10;
        ExponentialProfileWPI profile = new ExponentialProfileWPI(maxVel, maxAccel);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(3, 0);
        final double ETA_TOLERANCE = 0.02;
        double s = profile.solve(0.1, sample, end, 2.0, ETA_TOLERANCE);
        assertEquals(0.625, s, DELTA);
    }

    /** around 30 us at DT of 0.1. */
    @Test
    void testSolvePerformance() {
        double maxVel = 2;
        double maxAccel = 10;
        ExponentialProfileWPI profile = new ExponentialProfileWPI(maxVel, maxAccel);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(3, 0);
        final double ETA_TOLERANCE = 0.02;

        int N = 10000;
        double t0 = Takt.actual();
        for (int ii = 0; ii < N; ++ii) {
            profile.solve(0.1, sample, end, 1, ETA_TOLERANCE);
        }
        double t1 = Takt.actual();
        if (DEBUG)
            System.out.printf("duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
        if (DEBUG)
            System.out.printf("per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);
    }
}
