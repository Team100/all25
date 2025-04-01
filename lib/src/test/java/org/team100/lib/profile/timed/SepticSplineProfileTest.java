package org.team100.lib.profile.timed;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class SepticSplineProfileTest {
    private static final boolean DEBUG = false;
    private static final double kDelta = 0.001;

    @Test
    void testSimple() {
        SepticSplineProfile p = new SepticSplineProfile(2, 6);
        p.init(new Model100(), new Model100(1, 0));
        for (double t = 0; t < 2; t += 0.01) {
            Control100 c = p.sample(t);
            if (DEBUG)
                Util.printf("%12.3f %12.3f %12.3f %12.3f\n", t, c.x(), c.v(), c.a());
        }
    }

    /** Start == end, zero duration */
    @Test
    void testZero() {
        SepticSplineProfile p = new SepticSplineProfile(1, 1);
        p.init(new Model100(1, 0), new Model100(1, 0));
        Control100 c = p.sample(0);
        assertEquals(1, c.x());
        assertEquals(0, c.v());
        assertEquals(0, c.a());
    }

    void scan(SepticSplineProfile spline) {
        for (double t = 0; t <= spline.duration(); t += 0.001) {
            Control100 sample = spline.sample(t);
            double x = sample.x();
            double v = sample.v();
            double a = sample.a();
            if (DEBUG)
                Util.printf("%12.6f  %12.6f  %12.6f  %12.6f\n", t, x, v, a);
        }
    }

    double scanV(SepticSplineProfile profile) {
        double maxV = 0;
        for (double t = 0; t <= profile.duration(); t += 0.001) {
            double v = profile.sample(t).v();
            maxV = Math.max(maxV, Math.abs(v));
        }
        return maxV;
    }

    double scanA(SepticSplineProfile profile) {
        double maxA = 0;
        for (double t = 0; t <= profile.duration(); t += 0.001) {
            double a = profile.sample(t).a();
            maxA = Math.max(maxA, Math.abs(a));
        }
        return maxA;
    }

    /**
     * https://docs.google.com/spreadsheets/d/1EQd2n2MO53YZKoI3wZuXz7bBYFymacLprED5s_B-7pM/edit?pli=1&gid=0#gid=0
     */
    @Test
    void testMax1() {
        SepticSplineProfile p = new SepticSplineProfile(1, 1);
        p.init(0, 1, 0, 0, 0, 0, 0, 0);
        assertEquals(2.187, p.m_spline.maxV, kDelta);
        assertEquals(7.513, p.m_spline.maxA, kDelta);
        assertEquals(2.741, p.duration(), kDelta);
        // below limit, accel limited
        assertEquals(0.798, scanV(p), kDelta);
        // correct limit
        assertEquals(1, scanA(p), kDelta);
    }

    @Test
    void testMax1a() {
        // higher accel
        SepticSplineProfile p = new SepticSplineProfile(1, 2);
        p.init(0, 1, 0, 0, 0, 0, 0, 0);
        assertEquals(2.187, p.m_spline.maxV, kDelta);
        assertEquals(7.513, p.m_spline.maxA, kDelta);
        assertEquals(2.187, p.duration(), kDelta);
        // correct velocity limit
        assertEquals(1, scanV(p), kDelta);
        // below limit, velocity limited
        assertEquals(1.570, scanA(p), kDelta);
    }

    @Test
    void testMax2() {
        SepticSplineProfile p = new SepticSplineProfile(1, 1);
        // start with velocity
        // note these are [0,1] spline units not profile units :(
        p.init(0, 1, 1, 0, 0, 0, 0, 0);
        scan(p);
        assertEquals(1.66, p.m_spline.maxV, kDelta);
        assertEquals(5.028, p.m_spline.maxA, kDelta);
        assertEquals(2.242, p.duration(), kDelta);
        // below limit, accel limited
        assertEquals(0.74, scanV(p), kDelta);
        // correct accel limit
        assertEquals(1, scanA(p), kDelta);
    }

    @Test
    void testMax3() {
        SepticSplineProfile p = new SepticSplineProfile(1, 1);
        // TODO: spline units here are confusing
        p.init(0, 1, 0, 0, 2, 0, 0, 0);
        assertEquals(2.071, p.m_spline.maxV, kDelta);
        assertEquals(6.850, p.m_spline.maxA, kDelta);
        assertEquals(2.617, p.duration(), kDelta);
        assertEquals(0.791, scanV(p), kDelta);
        assertEquals(1, scanA(p), kDelta);
    }

    @Test
    void testMax4() {
        SepticSplineProfile p = new SepticSplineProfile(1, 1);
        p.init(0, 1, 0, 0, 0, 0, 1, 0);
        assertEquals(2.185, p.m_spline.maxV, kDelta);
        assertEquals(7.495, p.m_spline.maxA, kDelta);
        assertEquals(2.738, p.duration(), kDelta);
        assertEquals(0.798, scanV(p), kDelta);
        assertEquals(1, scanA(p), kDelta);
    }
}
