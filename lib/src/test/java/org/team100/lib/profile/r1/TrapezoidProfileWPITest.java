package org.team100.lib.profile.r1;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Takt;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

class TrapezoidProfileWPITest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    private void dump(double tt, Control100 sample) {
        if (DEBUG)
            System.out.printf("%f %f %f\n", tt, sample.x(), sample.v());
    }

    /**
     * this is a normal profile from 0 to 1, rest-to-rest, it's a triangle profile.
     */
    @Test
    void testTriangle() {
        TrapezoidProfileWPI profileX = new TrapezoidProfileWPI(5, 2);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(1, 0);

        double tt = 0;
        // the first sample is near the starting state
        dump(tt, sample);
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);
        assertEquals(0, sample.x(), DELTA);
        assertEquals(0.04, sample.v(), DELTA);

        // step to the middle of the profile
        for (double t = 0; t < 0.68; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        // halfway there, going fast
        assertEquals(0.5, sample.x(), 0.01);
        assertEquals(1.4, sample.v(), 0.01);

        // step to the end of the profile .. this was 0.72 before.
        for (double t = 0; t < 0.86; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(1.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    @Test
    void testSolve() {
        double maxVel = 2;
        double maxAccel = 10;
        TrapezoidProfileWPI profile = new TrapezoidProfileWPI(maxVel, maxAccel);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(3, 0);
        final double ETA_TOLERANCE = 0.02;
        double s = profile.solve(0.1, sample, end, 2.0, ETA_TOLERANCE);
        assertEquals(0.4375, s, DELTA);
    }

    /** Around 14 us per solve, using DT of 0.1. */
    @Test
    void testSolvePerformance() {
        double maxVel = 2;
        double maxAccel = 10;
        TrapezoidProfileWPI profile = new TrapezoidProfileWPI(maxVel, maxAccel);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(3, 0);
        final double ETA_TOLERANCE = 0.02;

        int N = 100000;
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
