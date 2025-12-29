package org.team100.lib.profile.r1;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;

public class IncrementalProfileTest implements Timeless {
    private static final boolean DEBUG = false;
    private final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    // 70 ns to get the ETA
    @Test
    void testETA() {
        Control100 initial = new Control100();
        Model100 goal = new Model100(1, 0);
        double expectedEta = 2.0;
        double s = 1.0;
        IncrementalProfile p = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        double diff = p.scale(s).simulateForETA(0.02, initial, goal) - expectedEta;
        // eta is indeed 2, it's a triangle path
        assertEquals(0, diff, 0.001);
        // same answer at 10x the step size
        double coarser = p.scale(s).simulateForETA(0.2, initial, goal) - expectedEta;
        assertEquals(0, coarser, 0.001);

        int N = 1000000;
        double t0 = Takt.actual();
        for (int ii = 0; ii < N; ++ii) {
            p.scale(s).simulateForETA(0.2, initial, goal);
        }
        double t1 = Takt.actual();
        if (DEBUG)
            System.out.printf("duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
        if (DEBUG)
            System.out.printf("per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);
    }

    // 3 us to get the eta at full resolution.
    // 10x coarser step means only 0.5 us
    @Test
    void testETA2() {
        Control100 initial = new Control100();
        Model100 goal = new Model100(1, 0);
        IncrementalProfile p = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        double eta = p.simulateForETA(0.02, initial, goal);
        assertEquals(2.000, eta, 0.001);
        // same answer at 10x the step size since the step happens to line up with the
        // triangle
        double coarser = p.simulateForETA(0.2, initial, goal);
        assertEquals(2.000, coarser, 0.001);

        int N = 1000000;
        double t0 = Takt.actual();
        for (int ii = 0; ii < N; ++ii) {
            // coarse step works fine, goes much faster.
            p.simulateForETA(0.2, initial, goal);
        }
        double t1 = Takt.actual();
        if (DEBUG)
            System.out.printf("duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
        if (DEBUG)
            System.out.printf("per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);
    }

    // exponential eta in 1.6 us using coarse step.
    @Test
    void testETA2Exponential() {
        Control100 initial = new Control100();
        Model100 goal = new Model100(1, 0);
        IncrementalProfile p = new ExponentialProfileWPI(1, 1);
        double eta = p.simulateForETA(0.02, initial, goal);
        assertEquals(2.180, eta, 0.001);
        // exponential is not the same with coarser step but it's close enough,
        // off by 0.02 sec.
        double coarser = p.simulateForETA(0.2, initial, goal);
        assertEquals(2.200, coarser, 0.001);

        int N = 1000000;
        double t0 = Takt.actual();
        for (int ii = 0; ii < N; ++ii) {
            // coarse step works fine, goes much faster.
            p.simulateForETA(0.2, initial, goal);
        }
        double t1 = Takt.actual();
        if (DEBUG)
            System.out.printf("duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
        if (DEBUG)
            System.out.printf("per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);
    }
}
