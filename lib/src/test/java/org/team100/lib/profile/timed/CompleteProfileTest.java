package org.team100.lib.profile.timed;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class CompleteProfileTest {
    private static final boolean DEBUG = true;
    private static final double DT = 0.02;
    private static final double kDelta = 0.001;

    /** Dump the sliding mode curve */
    @Test
    void testMode() {
        CompleteProfile p = new CompleteProfile(2, 6, 10, 40, 50, 0.001);
        if (DEBUG) {
            for (double x = -10; x < 10; x += 0.01) {
                Control100 sample = p.m_byDistance.get(x);
                Util.printf("%12.4f %12.4f %12.4f\n", sample.x(), sample.v(), sample.a());
            }
        }
    }

    @Test
    void testSimple() {
        CompleteProfile p = new CompleteProfile(3, 8, 12, 15, 50, 0.0001);
        final Model100 goal = new Model100(2, 0);
        Control100 c = new Control100();
        double t = 0;
        for (int i = 0; i < 200; ++i) {
            if (DEBUG)
                Util.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
            c = p.calculate(DT, c.model(), goal);
            t += DT;
        }
    }

    @Test
    void testSimple2() {
        CompleteProfile p = new CompleteProfile(2, 6, 10, 30, 50, 0.001);
        final Model100 goal = new Model100(-1, 0);
        Control100 c = new Control100();
        double t = 0;
        for (int i = 0; i < 100; ++i) {
            if (DEBUG)
                Util.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
            c = p.calculate(DT, c.model(), goal);
            t += DT;
        }
    }

    @Test
    void testMovingEntry() {
        CompleteProfile p = new CompleteProfile(2, 6, 10, 30, 50, 0.001);
        final Model100 goal = new Model100(1, 0);
        Control100 c = new Control100(0, -1);
        double t = 0;
        for (int i = 0; i < 100; ++i) {
            if (DEBUG)
                Util.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
            c = p.calculate(DT, c.model(), goal);
            t += DT;
        }
    }

    /** Moving goals are not allowed. */
    @Test
    void testMovingGoal() {
        CompleteProfile p = new CompleteProfile(2, 6, 10, 0.01);
        assertThrows(IllegalArgumentException.class,
                () -> p.calculate(0.02, new Model100(), new Model100(1, 1)));
    }

    /** How does interpolation work? */
    @Test
    void testKeyInterpolation1() {
        InterpolatingDoubleTreeMap m = new InterpolatingDoubleTreeMap();
        m.put(-1.0, 1.0);
        m.put(1.0, -1.0);
        // it takes the endpoint forever
        assertEquals(1, m.get(-3.0), kDelta);
        assertEquals(1, m.get(-2.0), kDelta);
        assertEquals(0, m.get(0.0), kDelta);
        assertEquals(-1, m.get(2.0), kDelta);
        assertEquals(-1, m.get(3.0), kDelta);
    }

    /** What if one of the points is really far away? */
    @Test
    void testKeyInterpolation2() {
        InterpolatingTreeMap<Double, Control100> m = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(),
                Control100::interpolate);
                // without this far-away point, the interpolator returns the endpoint at -1.
        m.put(-384400000.0, new Control100(-384400000, 1, 0));
        m.put(-1.0, new Control100(-1, 1, 0));
        m.put(1.0, new Control100(1, -1, 0));
        m.put(384400000.0, new Control100(384400000, -1, 0));
        // it takes the endpoint forever
        assertEquals(-3, m.get(-3.0).x(), kDelta);
        assertEquals(1, m.get(-3.0).v(), kDelta);
        assertEquals(-2, m.get(-2.0).x(), kDelta);
        assertEquals(0, m.get(0.0).x(), kDelta);
        assertEquals(2, m.get(2.0).x(), kDelta);
        assertEquals(3, m.get(3.0).x(), kDelta);
    }
}
