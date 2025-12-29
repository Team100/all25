package org.team100.lib.profile.r1;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;

/**
 * Illustrates how to coordinate multiple profiles to take the same amount of
 * time, by slowing down the faster one.
 * 
 * Coordinating multiple profiles with moving end-states involves a bit of
 * complexity as described in Lavalle, https://arxiv.org/pdf/2210.01744.pdf.
 * 
 * The approach here is simpler, and only works for at-rest end-states: in
 * short, you just slow down the max velocity and acceleration of the "faster"
 * profile. :-)
 * 
 * The WPI profile keeps the timing data around after the "calculate" method, so
 * we can look there; our own profile does not, so we need to modify it.
 */
class CoordinatedProfileTest implements Timeless {
    private static final boolean PRINT = false;

    private static final double PROFILE_TOLERANCE = 0.01;
    private static final double DELTA = 0.001;
    private static final double DT = 0.02;
    private final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    /**
     * Verify that the profile times are what we think they should be:
     * vel = 1, acc = 1
     * rest-to-rest 1 takes 2 sec with a triangle profile
     * rest-to-rest 2 takes 3 sec with a trapezoid.
     */
    @Test
    void testVerify() {
        final int maxVel = 1;
        final int maxAccel = 1;
        final double tolerance = 0.01;
        // two profiles with the same parameters
        TrapezoidIncrementalProfile p1 = new TrapezoidIncrementalProfile(logger, maxVel, maxAccel, tolerance);
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, maxVel, maxAccel, tolerance);
        // initial state at the origin at rest
        Model100 i1 = new Model100(0, 0);
        Model100 i2 = new Model100(0, 0);
        // final state at 1, at rest
        Model100 g1 = new Model100(1, 0);
        Model100 g2 = new Model100(2, 0);

        // how long does it take to get to the first goal?
        Control100 s1 = i1.control();
        double total_time = 0;
        double max_v = 0;
        for (int i = 0; i < 1000; ++i) {
            // next state
            s1 = p1.calculate(DT, s1, g1);
            total_time += DT;
            max_v = Math.max(max_v, s1.v());
            if (s1.model().near(g1, 0.01)) {
                if (PRINT)
                    System.out.println("at goal at t " + total_time);
                break;
            }
            if (PRINT)
                System.out.printf("%f %s\n", total_time, s1);
        }
        if (PRINT)
            System.out.println("max v " + max_v);
        // this is a triangle profile
        assertEquals(2.0, total_time, DELTA);

        // the second goal is farther away.
        Control100 s2 = i2.control();
        total_time = 0;
        max_v = 0;
        for (int i = 0; i < 1000; ++i) {
            // next state
            s2 = p2.calculate(DT, s2, g2);
            total_time += DT;
            max_v = Math.max(max_v, s2.v());
            if (s2.model().near(g2, 0.01)) {
                if (PRINT)
                    System.out.println("at goal at t " + total_time);
                break;
            }
            if (PRINT)
                System.out.printf("%f %s\n", total_time, s2);
        }
        if (PRINT)
            System.out.println("max v " + max_v);
        // this is a trapezoid profile
        assertEquals(3.0, total_time, DELTA);
    }

    /**
     * Same as above for WPI profile.
     */
    @Test
    void testVerifyWPI() {
        final int maxVel = 1;
        final int maxAccel = 1;
        // two profiles with the same parameters
        TrapezoidProfileWPI p1 = new TrapezoidProfileWPI(maxVel, maxAccel);
        TrapezoidProfileWPI p2 = new TrapezoidProfileWPI(maxVel, maxAccel);
        // initial state at the origin at rest
        Model100 i1 = new Model100(0, 0);
        Model100 i2 = new Model100(0, 0);
        // final state at 1, at rest
        Model100 g1 = new Model100(1, 0);
        Model100 g2 = new Model100(2, 0);

        // how long does it take to get to the first goal?
        Control100 s1 = i1.control();
        double total_time = 0;
        double max_v = 0;
        for (int i = 0; i < 1000; ++i) {
            // next state
            // note WPI doesn't produce accel in the profile. :-)
            s1 = p1.calculate(DT, s1, g1);
            total_time += DT;
            max_v = Math.max(max_v, s1.v());
            if (s1.model().near(g1, 0.01)) {
                if (PRINT)
                    System.out.println("at goal at t " + total_time);
                break;
            }
            if (PRINT)
                System.out.printf("%f %s\n", total_time, s1);
        }
        if (PRINT)
            System.out.println("max v " + max_v);
        // this is a triangle profile
        assertEquals(2.0, total_time, DELTA);

        // the second goal is farther away.
        Control100 s2 = i2.control();
        total_time = 0;
        max_v = 0;
        for (int i = 0; i < 1000; ++i) {
            // next state
            s2 = p2.calculate(DT, s2, g2);
            total_time += DT;
            max_v = Math.max(max_v, s2.v());
            if (s2.model().near(g2, 0.01)) {
                if (PRINT)
                    System.out.println("at goal at t " + total_time);
                break;
            }
            if (PRINT)
                System.out.printf("%f %s\n", total_time, s2);
        }
        if (PRINT)
            System.out.println("max v " + max_v);
        // this is a trapezoid profile
        assertEquals(3.0, total_time, DELTA);
    }

    /** Verify that the profile produces the correct ETA for these cases. */
    @Test
    void testETAs() {
        final int maxVel = 1;
        final int maxAccel = 1;
        final double tolerance = 0.01;
        // two profiles with the same parameters
        TrapezoidIncrementalProfile p1 = new TrapezoidIncrementalProfile(logger, maxVel, maxAccel, tolerance);
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, maxVel, maxAccel, tolerance);
        // initial state at the origin at rest
        Model100 i1 = new Model100(0, 0);
        Model100 i2 = new Model100(0, 0);
        // final state at 1, at rest
        Model100 g1 = new Model100(1, 0);
        Model100 g2 = new Model100(2, 0);

        double total_time = p1.simulateForETA(0.2, i1.control(), g1);
        assertEquals(2.0, total_time, DELTA);

        total_time = p2.simulateForETA(0.2, i2.control(), g2);
        assertEquals(3.0, total_time, DELTA);
    }

    /** Same as above for WPI */
    @Test
    void testETAsWPI() {
        final int maxVel = 1;
        final int maxAccel = 1;
        // two profiles with the same parameters
        TrapezoidProfileWPI p1 = new TrapezoidProfileWPI(maxVel, maxAccel);
        TrapezoidProfileWPI p2 = new TrapezoidProfileWPI(maxVel, maxAccel);
        // initial state at the origin at rest
        Model100 i1 = new Model100(0, 0);
        Model100 i2 = new Model100(0, 0);
        // final state at 1, at rest
        Model100 g1 = new Model100(1, 0);
        Model100 g2 = new Model100(2, 0);

        double total_time = p1.simulateForETA(0.2, i1.control(), g1);
        assertEquals(2.0, total_time, DELTA);

        total_time = p2.simulateForETA(0.2, i2.control(), g2);
        assertEquals(3.0, total_time, DELTA);
    }

    @Test
    void testCoordinatedProfiles() {
        final int maxVel = 1;
        final int maxAccel = 1;
        final double tolerance = 0.01;
        // default x and y profiles
        TrapezoidIncrementalProfile px = new TrapezoidIncrementalProfile(logger, maxVel, maxAccel, tolerance);
        TrapezoidIncrementalProfile py = new TrapezoidIncrementalProfile(logger, maxVel, maxAccel, tolerance);
        // initial x state is moving fast
        Control100 ix = new Control100(0, 1);
        // initial y state is stationary
        Control100 iy = new Control100(0, 0);
        // goal x state is still at the origin (i.e. a "slow and back up" profile)
        Model100 gx = new Model100(0, 0);
        // goal y state is not far
        Model100 gy = new Model100(0.5, 0);

        // the "default profiles" produce different ETA's
        double tx = px.simulateForETA(0.2, ix, gx);
        // approximate
        assertEquals(2.6, tx, DELTA);

        double ty = py.simulateForETA(0.2, iy, gy);
        // approximate
        assertEquals(1.6, ty, DELTA);

        // the slower ETA is the controlling one
        double slowETA = Math.max(tx, ty);
        assertEquals(2.6, slowETA, DELTA);

        // find the scale parameters for x and y.

        // in the X case, the given ETA is the default ETA
        double sx = px.solve(
                0.1, ix, gx, slowETA, DELTA);
        // in the Y case, it's slower
        double sy = py.solve(
                0.1, iy, gy, slowETA, DELTA);

        // this should be about 1.0
        assertEquals(0.937, sx, DELTA);
        assertEquals(0.281, sy, DELTA);

        // use the scale parameter to make adjusted profiles
        px = px.scale(sx);
        py = py.scale(sy);

        // then run the profiles to see where they end up

        Control100 stateX = ix;
        Control100 stateY = iy;
        @SuppressWarnings("unused")
        double total_time = 0;
        for (int i = 0; i < 1000; ++i) {
            total_time += DT;
            stateX = px.calculate(DT, stateX, gx);
            stateY = py.calculate(DT, stateY, gy);
            if (stateX.model().near(gx, PROFILE_TOLERANCE) && stateY.model().near(gy, PROFILE_TOLERANCE)) {
                if (PRINT)
                    System.out.println("at goal at t " + total_time);
                break;
            }
            if (PRINT)
                System.out.printf("%f %s %s\n", total_time, stateX, stateY);
        }
    }

}
