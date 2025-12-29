package org.team100.lib.profile.r1;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;

/**
 * Note many of these cases were adjusted slightly to accommodate the treatment
 * of max velocity.
 */
class TrapezoidIncrementalProfileTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final double TEN_MS = 0.01;
    private static final double DELTA = 0.001;
    private final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    private void dump(double tt, Control100 sample) {
        if (DEBUG)
            System.out.printf("%f %f %f %f\n", tt, sample.x(), sample.v(), sample.a());
    }

    /** Double integrator system simulator, kinda */
    static class Sim {
        /** this represents the system's inability to execute infinite jerk. */
        private static final double jerkLimit = 0.5;
        /** measured position */
        double y = 0;
        /** measured velocity */
        double yDot = 0;
        /** accel for imposing the jerk limit */
        double a = 0;

        /** evolve the system over the duration of this time step */
        void step(double u) {

            a = jerkLimit * a + (1 - jerkLimit) * u;

            y = y + yDot * 0.02 + 0.5 * a * 0.02 * 0.02;
            yDot = yDot + a * 0.02;
        }
    }

    @Test
    void testSolve() {
        double maxVel = 2;
        double maxAccel = 10;
        TrapezoidIncrementalProfile profile = new TrapezoidIncrementalProfile(logger, maxVel, maxAccel, 0.01);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(3, 0);
        final double ETA_TOLERANCE = 0.02;
        double s = profile.solve(0.1, sample, end, 2.0, ETA_TOLERANCE);
        assertEquals(0.4375, s, DELTA);
    }

    /** Around 25 us at 0.1 DT */
    @Test
    void testSolvePerformance() {
        double maxVel = 2;
        double maxAccel = 10;
        TrapezoidIncrementalProfile profile = new TrapezoidIncrementalProfile(logger, maxVel, maxAccel, 0.01);
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

    /**
     * see
     * https://docs.google.com/spreadsheets/d/19WbkNaxcRGHwYwLH1pu9ER3qxZrsYqDlZTdV-cmOM0I
     * 
     */
    @Test
    void testSample() {
        // see Spline1dTest.testSample()
        final IncrementalProfile p = new TrapezoidIncrementalProfile(logger, 2, 6, 0.01);
        Control100 setpoint = new Control100(0, 0);
        final Model100 goal = new Model100(1, 0);
        for (double t = 0; t < 1; t += 0.01) {
            setpoint = p.calculate(0.01, setpoint, goal);
            if (DEBUG) {
                double x = setpoint.x();
                double v = setpoint.v();
                double a = setpoint.a();
                double j = 0;
                System.out.printf("%8.3f %8.3f %8.3f %8.3f %8.3f\n",
                        t, x, v, a, j);
            }
        }
    }

    /** I think we're writing followers incorrectly, here's how to do it. */
    @Test
    void discreteTime1() {
        final IncrementalProfile profile = new TrapezoidIncrementalProfile(logger, 2, 1, 0.01);
        final Model100 initial = new Model100(0, 0);
        final Model100 goal = new Model100(1, 0);
        final double k1 = 5.0;
        final double k2 = 1.0;

        // initial state is motionless
        Sim sim = new Sim();
        sim.y = 0;
        sim.yDot = 0;
        double feedback = 0;
        Control100 setpointControl = new Control100();

        Model100 setpointModel = initial;
        if (DEBUG)
            System.out.printf(" t,      x,      v,      a,      y,      ydot,  fb,   eta\n");

        // log initial state
        if (DEBUG)
            System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                    0.0, setpointModel.x(), setpointModel.v(), 0.0, sim.y, sim.yDot, 0.0, 0.0);

        // eta to goal
        double etaS = 0;
        for (double currentTime = 0.0; currentTime < 3; currentTime += 0.02) {

            // at the beginning of the time step, we show the current measurement
            // and the setpoint calculated in the previous time step (which applies to this
            // one)
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                        currentTime,
                        setpointControl.x(),
                        setpointControl.v(),
                        setpointControl.a(),
                        sim.y,
                        sim.yDot,
                        feedback,
                        etaS);

            // compute feedback using the "previous" setpoint, which is for the current
            // instant
            feedback = k1 * (setpointModel.x() - sim.y)
                    + k2 * (setpointModel.v() - sim.yDot);

            setpointControl = profile.calculate(0.02, setpointModel.control(), goal);
            etaS = profile.simulateForETA(0.2, setpointModel.control(), goal);
            // this is the setpoint for the next time step
            setpointModel = setpointControl.model();

            // this is actuation for the next time step, using the feedback for the current
            // time, and feedforward for the next time step

            double u = setpointControl.a() + feedback;

            sim.step(u);
        }
    }

    /** What if the entry velocity is above the cruise velocity? */
    @Test
    void testTooHighEntryVelocity() {
        TrapezoidIncrementalProfile p = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        // initial state velocity is higher than profile cruise
        Control100 initial = new Control100(0, 2);
        // goal is achievable with constant max decel
        Model100 goal = new Model100(2, 0);
        Control100 r = p.calculate(0.02, initial, goal);
        double eta = p.simulateForETA(0.2, initial, goal);
        assertEquals(2, eta, DELTA);
        // 2m/s * 0.02s = ~0.04
        assertEquals(0.04, r.x(), DELTA);
        assertEquals(1.98, r.v(), DELTA);
        assertEquals(-1, r.a(), DELTA);
        for (double tt = 0.02; tt < 3; tt += 0.02) {
            r = p.calculate(0.02, r, goal);
            eta = p.simulateForETA(0.2, r, goal);
            dump(tt, r);
        }
        // at the goal
        assertEquals(0, eta, DELTA);
        assertEquals(2, r.x(), DELTA);
        assertEquals(0, r.v(), DELTA);
        assertEquals(0, r.a(), DELTA);
    }

    @Test
    void testTooHighEntryVelocityInReverse() {
        TrapezoidIncrementalProfile p = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        // initial state velocity is higher than profile cruise
        Control100 initial = new Control100(0, -2);
        // goal is achievable with constant max decel
        Model100 goal = new Model100(-2, 0);
        Control100 r = p.calculate(0.02, initial, goal);
        double eta = p.simulateForETA(0.2, initial, goal);
        assertEquals(2, eta, DELTA);
        // 2m/s * 0.02s = ~0.04
        assertEquals(-0.04, r.x(), DELTA);
        assertEquals(-1.98, r.v(), DELTA);
        assertEquals(1, r.a(), DELTA);
        for (double tt = 0.02; tt < 3; tt += 0.02) {
            r = p.calculate(0.02, r, goal);
            eta = p.simulateForETA(0.2, r, goal);
            dump(tt, r);
        }
        // at the goal
        assertEquals(0, eta, DELTA);
        assertEquals(-2, r.x(), DELTA);
        assertEquals(0, r.v(), DELTA);
        assertEquals(0, r.a(), DELTA);
    }

    @Test
    void testTooHighEntryVelocityCruising() {
        TrapezoidIncrementalProfile p = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        // initial state velocity is higher than profile cruise
        Control100 initial = new Control100(0, 2);
        // goal is achievable with max decel 1s, cruise 1s, max decel 1s
        Model100 goal = new Model100(3, 0);
        Control100 r = p.calculate(0.02, initial, goal);
        double eta = p.simulateForETA(0.2, initial, goal);
        // approximate
        assertEquals(3.2, eta, DELTA);
        // 2m/s * 0.02s = ~0.04
        assertEquals(0.04, r.x(), DELTA);
        assertEquals(1.98, r.v(), DELTA);
        assertEquals(-1, r.a(), DELTA);
        for (double tt = 0.02; tt < 4; tt += 0.02) {
            r = p.calculate(0.02, r, goal);
            eta = p.simulateForETA(0.2, r, goal);
            dump(tt, r);
        }
        // at the goal
        assertEquals(0, eta, DELTA);
        assertEquals(3, r.x(), DELTA);
        assertEquals(0, r.v(), DELTA);
        assertEquals(0, r.a(), DELTA);
    }

    /////////////////////////
    //
    // tests about duration
    //

    /** If you're at the goal, the ETA is zero. */
    @Test
    void testETAAtGoal() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        Control100 initial = new Control100(0, 0);
        Model100 goal = new Model100(0, 0); // same
        Control100 r = p2.calculate(0.02, initial, goal);
        // the next state is just the goal
        assertEquals(0, r.x(), DELTA);
        // and it takes zero time
        double eta = p2.simulateForETA(0.2, initial, goal);
        assertEquals(0, eta, DELTA);
    }

    /** Simple rest-to-rest case */
    @Test
    void testETARestToRest() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        Control100 initial = new Control100(0, 0);
        Model100 goal = new Model100(1, 0);
        Control100 s = p2.calculate(0.02, initial, goal);
        double eta = p2.simulateForETA(0.2, initial, goal);
        assertEquals(0.0002, s.x(), DELTA);
        assertEquals(0.02, s.v(), DELTA);
        assertEquals(1, s.a(), DELTA);
        // this is a triangular velocity profile
        assertEquals(2, eta, DELTA);
    }

    /**
     * How can we find parameters that *do* achieve the duration goal?
     * 
     * Scale acceleration. Scaling velocity is not as good, because you could scale
     * it instantly below the initial velocity.
     */
    @Test
    void testETASolve() {
        Control100 initial = new Control100(0, 0);
        Model100 goal = new Model100(1, 0);
        TrapezoidIncrementalProfile p = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        // this this is the default eta above, so s = 1.0.
        double s = p.solve(0.1, initial, goal, 2, DELTA);
        assertEquals(1.0, s, DELTA);
        s = p.solve(0.1, initial, goal, 3, DELTA);
        // approximate
        assertEquals(0.4375, s, DELTA);
        s = p.solve(0.1, initial, goal, 4, DELTA);
        // approximate
        assertEquals(0.25, s, DELTA);
        s = p.solve(0.1, initial, goal, 8, DELTA);
        // approximate
        assertEquals(0.051, s, DELTA);
    }

    /**
     * I happened to notice that the eta solver is sometimes wrong
     */
    @Test
    void testBrokenSolve1() {
        // here we just slow down and proceed
        // we're heading fast towards the goal.
        // first brake for 0.449 sec, which puts us at 1.188
        // then cruise at -0.01 until 0, so 118.8 sec
        // total is about 119.2
        // System.out.println("**** first calculate the ETA");
        // very low max vel
        double maxV = 0.01;
        // high max accel
        double maxA = 10;
        double tol = 0.01;
        TrapezoidIncrementalProfile px = new TrapezoidIncrementalProfile(logger, maxV, maxA, tol);
        Control100 initial = new Control100(2.2, -4.5);
        Model100 goal = new Model100(0, 0);
        double eta = px.simulateForETA(0.2, initial, goal);
        // the simulator times out at 10 sec
        assertTrue(Double.isInfinite(eta));

        // System.out.println("**** then find S for this very same ETA");
        double s = px.solve(0.1, initial, goal, 119.2, DELTA);

        // previously the "s" value here was 0.292, not 1.0, even though we're using
        // the very same parameters. Why?
        //
        // there are two ways to achieve the ETA.
        // one way is to brake quickly and cruise for a long time.
        // the other way is to brake more slowly, make a u-turn, and cruising back.
        //
        // I fixed this by making the solver notice if one of the endpoints is the
        // solution.

        assertEquals(1, s, DELTA);
    }

    @Test
    void testBrokenSolve2() {
        // this is a u-turn
        // brake for 0.459, puts us at 6.058
        // cruise at 0.01, so 605.8 sec
        // around 606.2 total
        // System.out.println("**** first calculate the ETA");
        // very low max vel
        double maxV = 0.01;
        // high max accel
        double maxA = 10;
        double tol = 0.01;
        TrapezoidIncrementalProfile px = new TrapezoidIncrementalProfile(logger, maxV, maxA, tol);
        // heading away from the goal, this is a very slow u-turn
        Control100 initial = new Control100(5.0, 4.6);
        Model100 goal = new Model100(0, 0);
        double eta = px.simulateForETA(0.2, initial, goal);
        assertTrue(Double.isInfinite(eta));

        // System.out.println("**** then find S for this very same ETA");
        double s = px.solve(0.1, initial, goal, 606.261, DELTA);
        // this is correct.
        assertEquals(1, s, DELTA);
    }

    @Test
    void testETASolveStationary() {
        Control100 initial = new Control100(0, 0);
        Model100 goal = new Model100(0, 0);
        TrapezoidIncrementalProfile p = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        // this this is the default eta above, so s = 1.0.
        double s = p.solve(0.1, initial, goal, 2, DELTA);
        assertEquals(1.0, s, DELTA);
    }

    /** ETA is not a trivial function of V and A */
    @Test
    void testETARestToRestScaled1() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 0.5, 1, 0.01);
        Control100 initial = new Control100(0, 0);
        Model100 goal = new Model100(1, 0);
        Control100 s = p2.calculate(0.02, initial, goal);
        assertEquals(0.0, s.x(), DELTA);
        assertEquals(0.02, s.v(), DELTA);
        assertEquals(1, s.a(), DELTA);
        double eta = p2.simulateForETA(0.2, initial, goal);
        // approximate
        assertEquals(2.6, eta, DELTA);
    }

    /** ETA is not a trivial function of V and A */
    @Test
    void testETARestToRestScaled2() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 0.5, 0.5, 0.01);
        Control100 initial = new Control100(0, 0);
        Model100 goal = new Model100(1, 0);
        Control100 s = p2.calculate(0.02, initial, goal);
        assertEquals(0.0, s.x(), DELTA);
        assertEquals(0.01, s.v(), DELTA);
        assertEquals(0.5, s.a(), DELTA);
        // this is a trapezoidal velocity profile
        double eta = p2.simulateForETA(0.2, initial, goal);
        assertEquals(3, eta, DELTA);
    }

    /** ETA is not a trivial function of V and A */
    @Test
    void testETARestToRestScaled3() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 0.25, 0.25, 0.01);
        Control100 initial = new Control100(0, 0);
        Model100 goal = new Model100(1, 0);
        Control100 s = p2.calculate(0.02, initial, goal);
        assertEquals(0.0, s.x(), DELTA);
        assertEquals(0.005, s.v(), DELTA);
        assertEquals(0.25, s.a(), DELTA);
        // this is a trapezoidal velocity profile
        double eta = p2.simulateForETA(0.2, initial, goal);
        assertEquals(5, eta, DELTA);
    }

    /** Initially at max V, cruise and then slow to a stop */
    @Test
    void testETACruise() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        Control100 initial = new Control100(0, 1); // cruising at maxV
        Model100 goal = new Model100(1, 0); // want to go 1m, so cruise for 0.5m, 0.5s, then slow for 1s
        Control100 s = p2.calculate(0.02, initial, goal);
        // the next state should be a small step in the direction of the goal
        assertEquals(0.02, s.x(), DELTA);
        // at the initial velocity
        assertEquals(1, s.v(), DELTA);
        // still cruising for now
        assertEquals(0, s.a(), DELTA);
        // cruise for 0.5s, then slow for 1s
        double eta = p2.simulateForETA(0.2, initial, goal);
        // approximate
        assertEquals(1.6, eta, DELTA);
    }

    /** Initially at max V, slow immediately */
    @Test
    void testETACruiseGMinus() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        Control100 initial = new Control100(0, 1); // cruising at maxV
        Model100 goal = new Model100(0.5, 0); // want to go 0.5m, so we're on G-
        Control100 s = p2.calculate(0.02, initial, goal);
        // still moving at roughly initial v
        assertEquals(0.02, s.x(), DELTA);
        // slowing
        assertEquals(0.98, s.v(), DELTA);
        // braking
        assertEquals(-1, s.a(), DELTA);
        // will take 1s
        double eta = p2.simulateForETA(0.2, initial, goal);
        assertEquals(1, eta, DELTA);
    }

    /** Initially at cruise, goal is the same position */
    @Test
    void testETAReverse() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        Control100 initial = new Control100(0, 1);
        Model100 goal = new Model100(0, 0);
        Control100 s = p2.calculate(0.02, initial, goal);
        // initial velocity carries us forward
        assertEquals(0.02, s.x(), DELTA);
        // starting to slow down
        assertEquals(0.98, s.v(), DELTA);
        // max braking
        assertEquals(-1, s.a(), DELTA);
        // then slow to a stop for 1s (0.5m), then back up and stop (1.4s) so 2.414
        // total
        double eta = p2.simulateForETA(0.2, initial, goal);
        // approximate
        assertEquals(2.6, eta, DELTA);
    }

    /** Same as above in the other direction */
    @Test
    void testETACruiseMinus() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        Control100 initial = new Control100(0, -1);
        Model100 goal = new Model100(-1, 0);
        Control100 s = p2.calculate(0.02, initial, goal);
        assertEquals(-0.02, s.x(), DELTA);
        assertEquals(-1, s.v(), DELTA);
        assertEquals(0, s.a(), DELTA);
        double eta = p2.simulateForETA(0.2, initial, goal);
        // approximate
        assertEquals(1.6, eta, DELTA);
    }

    /** Same as above in the other direction */
    @Test
    void testETACruiseMinusGPlus() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 1, 1, 0.01);
        Control100 initial = new Control100(0, -1);
        Model100 goal = new Model100(-0.5, 0);
        Control100 s = p2.calculate(0.02, initial, goal);
        assertEquals(-0.02, s.x(), DELTA);
        assertEquals(-0.98, s.v(), DELTA);
        assertEquals(1, s.a(), DELTA);
        double eta = p2.simulateForETA(0.2, initial, goal);
        assertEquals(1, eta, DELTA);
    }

    //////////////////////
    //
    // tests about the new profile
    //

    /** Now we expose acceleration in the profile state, so make sure it's right. */
    @Test
    void testAccel1() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        Model100 initial = new Model100(0, 0);
        Model100 goal = new Model100(1, 0);
        Control100 s = p2.calculate(0.02, initial.control(), goal);
        // 0.5 * 2 * 0.02 * 0.02 = 0.0004
        assertEquals(0.0004, s.x(), 0.000001);
        // 2 * 0.02 = 0.04
        assertEquals(0.04, s.v(), 0.000001);
        // I+ a=2
        assertEquals(2, s.a(), 0.000001);
    }

    @Test
    void testAccel2() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        // inverted
        Model100 initial = new Model100(0, 0);
        Model100 goal = new Model100(-1, 0);
        Control100 s = p2.calculate(0.02, initial.control(), goal);
        // 0.5 * 2 * 0.02 * 0.02 = 0.0004
        assertEquals(-0.0004, s.x(), 0.000001);
        // 2 * 0.02 = 0.04
        assertEquals(-0.04, s.v(), 0.000001);
        // I+ a=2
        assertEquals(-2, s.a(), 0.000001);
    }

    @Test
    void testAccel3() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        // cruising
        Model100 initial = new Model100(0, 3);
        Model100 goal = new Model100(10, 0);
        Control100 s = p2.calculate(1, initial.control(), goal);
        // cruising at 3 for 1
        assertEquals(3, s.x(), 0.001);
        // cruising
        assertEquals(3, s.v(), 0.000001);
        // cruising => zero accel
        assertEquals(0, s.a(), 0.000001);
    }

    @Test
    void testIntercepts() {
        TrapezoidIncrementalProfile p = new TrapezoidIncrementalProfile(logger.name("one"), 5, 0.5, 0.01);
        Control100 s = new Control100(1, 1);
        assertEquals(0, p.c_plus(s), DELTA);
        assertEquals(2, p.c_minus(s), DELTA);

        // more accel
        p = new TrapezoidIncrementalProfile(logger.name("two"), 5, 1, 0.01);
        s = new Control100(1, 1);
        // means less offset
        assertEquals(0.5, p.c_plus(s), DELTA);
        assertEquals(1.5, p.c_minus(s), DELTA);

        // negative velocity, result should be the same.
        p = new TrapezoidIncrementalProfile(logger.name("three"), 5, 1, 0.01);
        s = new Control100(1, -1);
        // means less offset
        assertEquals(0.5, p.c_plus(s), DELTA);
        assertEquals(1.5, p.c_minus(s), DELTA);
    }

    // see studies/rrts TestRRTStar7
    @Test
    void testInterceptsFromRRT() {
        TrapezoidIncrementalProfile p = new TrapezoidIncrementalProfile(logger.name("one"), 5, 1, 0.01);

        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger.name("two"), 5, 2, 0.01);

        assertEquals(0, p.c_minus(new Control100(0, 0)), 0.001);
        assertEquals(0, p.c_plus(new Control100(0, 0)), 0.001);

        assertEquals(0.5, p.c_minus(new Control100(0, 1)), 0.001);
        assertEquals(-0.5, p.c_plus(new Control100(0, 1)), 0.001);

        assertEquals(1.5, p.c_minus(new Control100(1, 1)), 0.001);
        assertEquals(0.5, p.c_plus(new Control100(1, 1)), 0.001);

        assertEquals(-0.5, p.c_minus(new Control100(-1, 1)), 0.001);
        assertEquals(-1.5, p.c_plus(new Control100(-1, 1)), 0.001);

        assertEquals(0.5, p.c_minus(new Control100(0, -1)), 0.001);
        assertEquals(-0.5, p.c_plus(new Control100(0, -1)), 0.001);

        assertEquals(2, p.c_minus(new Control100(0, 2)), 0.001);
        assertEquals(-2, p.c_plus(new Control100(0, 2)), 0.001);

        assertEquals(0.25, p2.c_minus(new Control100(0, 1)), 0.001);
        assertEquals(-0.25, p2.c_plus(new Control100(0, 1)), 0.001);

        // these are cases for the switching point test below
        // these curves don't intersect at all
        assertEquals(-1, p.c_minus(new Control100(-3, 2)), 0.001);
        assertEquals(0, p.c_plus(new Control100(2, 2)), 0.001);

        // these curves intersect exactly once at the origin
        assertEquals(0, p.c_minus(new Control100(-2, 2)), 0.001);
        assertEquals(0, p.c_plus(new Control100(2, 2)), 0.001);

        // these two curves intersect twice, once at (0.5,1) and once at (0.5,-1)
        assertEquals(1, p.c_minus(new Control100(-1, 2)), 0.001);
        assertEquals(0, p.c_plus(new Control100(2, 2)), 0.001);
    }

    @Test
    void testQSwitch() {
        TrapezoidIncrementalProfile p = new TrapezoidIncrementalProfile(logger.name("one"), 5, 1, 0.01);

        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger.name("two"), 5, 2, 0.01);

        assertEquals(0.375, p2.qSwitchIplusGminus(new Control100(0, 0), new Model100(0.5, 1.0)), 0.001);
        assertEquals(0.125, p2.qSwitchIminusGplus(new Control100(0, 0), new Model100(0.5, 1.0)), 0.001);

        assertEquals(-0.5, p.qSwitchIplusGminus(new Control100(-3, 2), new Model100(2, 2)), 0.001);
        assertEquals(0, p.qSwitchIplusGminus(new Control100(-2, 2), new Model100(2, 2)), 0.001);
        assertEquals(0.5, p.qSwitchIplusGminus(new Control100(-1, 2), new Model100(2, 2)), 0.001);

        assertEquals(-0.5, p.qSwitchIminusGplus(new Control100(2, -2), new Model100(-3, -2)), 0.001);
        assertEquals(0.0, p.qSwitchIminusGplus(new Control100(2, -2), new Model100(-2, -2)), 0.001);
        assertEquals(0.5, p.qSwitchIminusGplus(new Control100(2, -2), new Model100(-1, -2)), 0.001);

        // these are all a little different just to avoid zero as the answer
        assertEquals(0.5, p.qSwitchIplusGminus(new Control100(2, 2), new Model100(-1, 2)), 0.001);
        assertEquals(0.5, p.qSwitchIplusGminus(new Control100(-1, 2), new Model100(2, -2)), 0.001);
        assertEquals(0.5, p.qSwitchIminusGplus(new Control100(2, 2), new Model100(-1, 2)), 0.001);
        assertEquals(0.5, p.qSwitchIminusGplus(new Control100(-1, 2), new Model100(2, -2)), 0.001);
    }

    /** Verify some switching velocity cases */
    @Test
    void testQDotSwitch2() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12)
        assertEquals(3.464, p2.qDotSwitchIplusGminus(new Control100(-2, 2), new Model100(2, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8)
        assertEquals(2.828, p2.qDotSwitchIplusGminus(new Control100(-1, 2), new Model100(1, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6)
        assertEquals(2.449, p2.qDotSwitchIplusGminus(new Control100(-0.5, 2), new Model100(0.5, 2)), 0.001);
        // the same point
        assertEquals(2.000, p2.qDotSwitchIplusGminus(new Control100(0, 2), new Model100(0, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Control100(0.5, 2), new Model100(-0.5, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Control100(1, 2), new Model100(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Control100(2, 2), new Model100(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Control100(-2, 2), new Model100(2, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Control100(-1, 2), new Model100(1, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Control100(-0.5, 2), new Model100(0.5, 2)), 0.001);
        // the same point
        assertEquals(2.0, p2.qDotSwitchIminusGplus(new Control100(0, 2), new Model100(0, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6), negative arm
        assertEquals(-2.449, p2.qDotSwitchIminusGplus(new Control100(0.5, 2), new Model100(-0.5, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8), negative arm
        assertEquals(-2.828, p2.qDotSwitchIminusGplus(new Control100(1, 2), new Model100(-1, 2)), 0.001);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12) but the negative arm
        assertEquals(-3.464, p2.qDotSwitchIminusGplus(new Control100(2, 2), new Model100(-2, 2)), 0.001);

    }

    @Test
    void testQDotSwitch2a() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12)
        assertEquals(3.464, p2.qDotSwitchIplusGminus(new Control100(-2, 2), new Model100(2, -2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8)
        assertEquals(2.828, p2.qDotSwitchIplusGminus(new Control100(-1, 2), new Model100(1, -2)), 0.001);
        assertEquals(2.449, p2.qDotSwitchIplusGminus(new Control100(-0.5, 2), new Model100(0.5, -2)), 0.001);
        // the path switches immediately
        assertEquals(2.000, p2.qDotSwitchIplusGminus(new Control100(0, 2), new Model100(0, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Control100(0.5, 2), new Model100(-0.5, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Control100(1, 2), new Model100(-1, -2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Control100(2, 2), new Model100(-2, -2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Control100(-2, 2), new Model100(2, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Control100(-1, 2), new Model100(1, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Control100(-0.5, 2), new Model100(0.5, -2)), 0.001);
        // switching at the goal
        assertEquals(-2.000, p2.qDotSwitchIminusGplus(new Control100(0, 2), new Model100(0, -2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6), negative arm
        assertEquals(-2.449, p2.qDotSwitchIminusGplus(new Control100(0.5, 2), new Model100(-0.5, -2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8), negative arm
        assertEquals(-2.828, p2.qDotSwitchIminusGplus(new Control100(1, 2), new Model100(-1, -2)), 0.001);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12) but the negative arm
        assertEquals(-3.464, p2.qDotSwitchIminusGplus(new Control100(2, 2), new Model100(-2, -2)), 0.001);

    }

    @Test
    void testQDotSwitch2b() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12)
        assertEquals(3.464, p2.qDotSwitchIplusGminus(new Control100(-2, -2), new Model100(2, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8)
        assertEquals(2.828, p2.qDotSwitchIplusGminus(new Control100(-1, -2), new Model100(1, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6)
        assertEquals(2.449, p2.qDotSwitchIplusGminus(new Control100(-0.5, -2), new Model100(0.5, 2)), 0.001);
        // switches at G
        assertEquals(2.000, p2.qDotSwitchIplusGminus(new Control100(0, -2), new Model100(0, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Control100(0.5, -2), new Model100(-0.5, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Control100(1, -2), new Model100(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Control100(2, -2), new Model100(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Control100(-2, -2), new Model100(2, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Control100(-1, -2), new Model100(1, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Control100(-0.5, -2), new Model100(0.5, -2)), 0.001);
        // switches at I
        assertEquals(-2.000, p2.qDotSwitchIminusGplus(new Control100(0, -2), new Model100(0, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6), negative arm
        assertEquals(-2.449, p2.qDotSwitchIminusGplus(new Control100(0.5, -2), new Model100(-0.5, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8), negative arm
        assertEquals(-2.828, p2.qDotSwitchIminusGplus(new Control100(1, -2), new Model100(-1, 2)), 0.001);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12) but the negative arm
        assertEquals(-3.464, p2.qDotSwitchIminusGplus(new Control100(2, -2), new Model100(-2, 2)), 0.001);
    }

    @Test
    void testOneLongT() {
        // if we supply a very long dt, we should end up at the goal
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        Model100 initial = new Model100(0, 0);
        // goal is far, requires (brief) cruising
        Model100 goal = new Model100(5, 0);
        Control100 s = p2.calculate(10, initial.control(), goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.00001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void testOneLongTReverse() {
        // if we supply a very long dt, we should end up at the goal
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        Model100 initial = new Model100(0, 0);
        // goal is far, requires (brief) cruising
        Model100 goal = new Model100(-5, 0);
        Control100 s = p2.calculate(10, initial.control(), goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.00001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void testManyLongT() {
        // if we supply a very long dt, we should end up at the goal
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        Random random = new Random();
        for (int i = 0; i < 10000; ++i) {
            // random states in the square between (-2,-2) and (2,2)
            Model100 initial = new Model100(4.0 * random.nextDouble() - 2.0, 4.0 * random.nextDouble() - 2.0);
            Model100 goal = new Model100(4.0 * random.nextDouble() - 2.0, 4.0 * random.nextDouble() - 2.0);
            Control100 s = p2.calculate(10, initial.control(), goal);
            // it always gets exactly to the goal
            assertEquals(goal.x(), s.x(), 0.00001);
            assertEquals(goal.v(), s.v(), 0.000001);
        }
    }

    @Test
    void reciprocal() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        Model100 initial = new Model100(-1, 1);
        Model100 goal = new Model100(-1, -1);
        Control100 s = p2.calculate(10, initial.control(), goal);
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void endEarly() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        // in this case, t1 for I+G- is 0, and i think I-G+ is doing the wrong thing.
        // the delta v is 1, accel is 2, so this is a 0.5s solution.
        Control100 initial = new Control100(-1, 2);
        Model100 goal = new Model100(-0.25, 1);

        // in this case the I-G+ path switching point is the reciprocal, which isn't
        // what we want,
        // but it shouldn't matter because the I+G- switching point is I, so we should
        // choose that.
        double qdot = p2.qDotSwitchIminusGplus(initial, goal);
        assertEquals(-1, qdot, 0.001);
        double qdotIpGm = p2.qDotSwitchIplusGminus(initial, goal);
        assertEquals(2, qdotIpGm, 0.001);
        // this is the long way around
        double t1ImGp = p2.t1IminusGplus(initial, goal);
        assertEquals(1.5, t1ImGp, 0.001);
        // since this is zero we should choose I+G-
        double t1IpGm = p2.t1IplusGminus(initial, goal);
        assertEquals(0, t1IpGm, 0.001);

        Control100 s = p2.calculate(10, initial, goal);
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    // like above but with reciprocal starting point
    @Test
    void endEarly2() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);

        Control100 initial = new Control100(-1, -2);
        Model100 goal = new Model100(-0.25, 1);

        double qdot = p2.qDotSwitchIminusGplus(initial, goal);
        assertEquals(Double.NaN, qdot, 0.001);

        double qdotIpGm = p2.qDotSwitchIplusGminus(initial, goal);
        assertEquals(2, qdotIpGm, 0.001);

        double t1ImGp = p2.t1IminusGplus(initial, goal);
        assertEquals(Double.NaN, t1ImGp, 0.001);

        double t1IpGm = p2.t1IplusGminus(initial, goal);
        assertEquals(2, t1IpGm, 0.001);

        Control100 s = p2.calculate(10, initial, goal);
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void anotherCase() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        Control100 initial = new Control100(1.127310, -0.624930);
        Model100 goal = new Model100(1.937043, 0.502350);
        Control100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void yetAnother() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        Control100 initial = new Control100(-1.178601, -1.534504);
        Model100 goal = new Model100(-0.848954, -1.916583);
        Control100 s = p2.calculate(10, initial, goal);
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase() {
        // this is an I-G+ path
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        Model100 initial = new Model100(1.655231, 1.967906);
        Model100 goal = new Model100(0.080954, -1.693829);
        Control100 s = p2.calculate(10, initial.control(), goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase2() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);

        Control100 initial = new Control100(1.747608, -0.147275);
        Model100 goal = new Model100(1.775148, 0.497717);

        double cplus = p2.c_plus(initial);
        assertEquals(1.742, cplus, 0.001);
        double cminus = p2.c_minus(goal.control());
        assertEquals(1.837, cminus, 0.001);

        double qSwitch = p2.qSwitchIplusGminus(initial, goal);
        assertEquals(1.789, qSwitch, 0.001);

        double nono = p2.qSwitchIminusGplus(initial, goal);
        assertEquals(1.733, nono, 0.001);

        // G is +v from I, so should be to the right of I+ to have a solution.
        // I+ intercept is 1.742, goal.v is 0.497717
        // surface x is 1.742 + 0.497717^2/(2*2) = 1.803930
        // goal.p is 1.775148 which is to the left so there is no I+G- solution.
        double qDotSwitch = p2.qDotSwitchIplusGminus(initial, goal);
        assertEquals(Double.NaN, qDotSwitch, 0.0001);

        // the I-G+ path has a small negative switching velocity
        double qDotSwitchImGp = p2.qDotSwitchIminusGplus(initial, goal);
        assertEquals(-0.282180, qDotSwitchImGp, 0.0001);

        // there is no I+G- path
        double t1 = p2.t1IplusGminus(initial, goal);
        assertEquals(Double.NaN, t1, 0.000001);
        double t1a = p2.t1IminusGplus(initial, goal);
        assertEquals(0.067452, t1a, 0.000001);

        Control100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase3() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        Model100 initial = new Model100(0.985792, 1.340926);
        Model100 goal = new Model100(-0.350934, -1.949649);
        Control100 s = p2.calculate(10, initial.control(), goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase4() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        Model100 initial = new Model100(0, 1);
        Model100 goal = new Model100(0, -1);
        Control100 s = p2.calculate(10, initial.control(), goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase2a() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        Model100 initial = new Model100(1.747608, -0.147275);
        Model100 goal = new Model100(1.775148, 0.497717);
        Control100 s = p2.calculate(10, initial.control(), goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    /** verify time to velocity limit */
    @Test
    void testVT() {
        // lower max V than the other cases here
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        // initial is (-2,2), vmax is 3, u is 2, so time to limit is 0.5.
        // at 0.5, v=2+2*0.5=3. x=-2+2*0.5+0.5*2*(0.5)^2 = -2+1+0.25=-0.75
        // so this is right at the limit, we should just proceed.
        Control100 s = p2.calculate(0.02, new Control100(-0.75, 3.00), new Model100(2, 2));
        // at vmax for 0.02, -0.75+3*0.02 = exactly -0.69, no t^2 term
        assertEquals(-0.6900, s.x(), 0.0001);
        // should continue at vmax, not go faster
        assertEquals(3.00, s.v(), 0.001);

        // same thing, inverted
        s = p2.calculate(0.02, new Control100(0.75, -3.00), new Model100(-2, -2));
        assertEquals(0.6900, s.x(), 0.0001);
        assertEquals(-3.00, s.v(), 0.001);
    }

    @Test
    void testVT2() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);

        // if we're *near* the limit then there should be two segments.
        Control100 s = p2.calculate(0.02, new Control100(-0.78, 2.98), new Model100(2, 2));
        // follow the profile for about 0.01, then the limit for another 0.01
        // at vmax for 0.02, -0.75+3*0.02 = exactly -0.69, no t^2 term
        assertEquals(-0.7200, s.x(), 0.0001);
        // end up at exactly vmax
        assertEquals(3.00, s.v(), 0.001);

        // same, inverted.
        s = p2.calculate(0.02, new Control100(0.78, -2.98), new Model100(-2, -2));
        assertEquals(0.7200, s.x(), 0.0001);
        assertEquals(-3.00, s.v(), 0.001);
    }

    @Test
    void testVT3() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);

        // if we're at the limit but right at the end, we should join G-.
        Control100 s = p2.calculate(0.02, new Control100(0.75, 3.00), new Model100(2, 2));
        // dx = 0.06 - 0.0004
        assertEquals(0.8096, s.x(), 0.0001);
        // dv = 0.04
        assertEquals(2.96, s.v(), 0.001);

        // same, inverted
        s = p2.calculate(0.02, new Control100(-0.75, -3.00), new Model100(-2, -2));
        assertEquals(-0.8096, s.x(), 0.0001);
        assertEquals(-2.96, s.v(), 0.001);
    }

    @Test
    void testVT4() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 3, 2, 0.01);
        // if we're *near* the end, there should be two segments.
        // 0.75-0.01*3
        Control100 s = p2.calculate(0.02, new Control100(0.72, 3.00), new Model100(2, 2));
        // so for the second 0.01 we should be slowing down
        // x = 0.75 + 0.03 - 0.0001
        // this needs to be exact; we're not taking the tswitch path
        assertEquals(0.77993, s.x(), 0.0001);
        // v = 3 - 0.02
        assertEquals(2.98, s.v(), 0.001);

        // same thing, inverted
        s = p2.calculate(0.02, new Control100(-0.72, -3.00), new Model100(-2, -2));
        // for the second segment we should be speeding up
        // x = -0.75 - 0.03 + 0.0001
        assertEquals(-0.7799, s.x(), 0.0001);
        // dv = 0.02
        assertEquals(-2.98, s.v(), 0.001);

    }

    /** Verify the time to the switching point via each path */
    @Test
    void testT() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);
        // dv=1.464, a=2
        assertEquals(0.732, p2.t1IplusGminus(new Control100(-2, 2), new Model100(2, 2)), 0.001);
        // dv=0.828, a=2
        assertEquals(0.414, p2.t1IplusGminus(new Control100(-1, 2), new Model100(1, 2)), 0.001);
        // dv = 0.449, a=2
        assertEquals(0.225, p2.t1IplusGminus(new Control100(-0.5, 2), new Model100(0.5, 2)), 0.001);
        // dv = 0
        assertEquals(0.000, p2.t1IplusGminus(new Control100(0, 2), new Model100(0, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.t1IplusGminus(new Control100(0.5, 2), new Model100(-0.5, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.t1IplusGminus(new Control100(1, 2), new Model100(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.t1IplusGminus(new Control100(2, 2), new Model100(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.t1IminusGplus(new Control100(-2, 2), new Model100(2, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.t1IminusGplus(new Control100(-1, 2), new Model100(1, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.t1IminusGplus(new Control100(-0.5, 2), new Model100(0.5, 2)), 0.001);
        // dv = 0
        assertEquals(0.000, p2.t1IminusGplus(new Control100(0, 2), new Model100(0, 2)), 0.001);
        // dv = -4.449, a=2
        assertEquals(2.225, p2.t1IminusGplus(new Control100(0.5, 2), new Model100(-0.5, 2)), 0.001);
        // dv = -4.828, a=2
        assertEquals(2.414, p2.t1IminusGplus(new Control100(1, 2), new Model100(-1, 2)), 0.001);
        // dv = -5.464
        assertEquals(2.732, p2.t1IminusGplus(new Control100(2, 2), new Model100(-2, 2)), 0.001);
    }

    @Test
    void testTa() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);

        // dv=1.464
        assertEquals(0.732, p2.t1IplusGminus(new Control100(-2, 2), new Model100(2, -2)), 0.001);
        // dv=0.828
        assertEquals(0.414, p2.t1IplusGminus(new Control100(-1, 2), new Model100(1, -2)), 0.001);
        assertEquals(0.225, p2.t1IplusGminus(new Control100(-0.5, 2), new Model100(0.5, -2)), 0.001);
        // the path switches immediately
        assertEquals(0.000, p2.t1IplusGminus(new Control100(0, 2), new Model100(0, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.t1IplusGminus(new Control100(0.5, 2), new Model100(-0.5, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.t1IplusGminus(new Control100(1, 2), new Model100(-1, -2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.t1IplusGminus(new Control100(2, 2), new Model100(-2, -2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.t1IminusGplus(new Control100(-2, 2), new Model100(2, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.t1IminusGplus(new Control100(-1, 2), new Model100(1, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Control100(-0.5, 2), new Model100(0.5, -2)), 0.001);
        // switching at the goal, dv=4, a=2
        assertEquals(2.000, p2.t1IminusGplus(new Control100(0, 2), new Model100(0, -2)), 0.001);
        // dv=-4.449
        assertEquals(2.225, p2.t1IminusGplus(new Control100(0.5, 2), new Model100(-0.5, -2)), 0.001);
        // dv=-4.828
        assertEquals(2.414, p2.t1IminusGplus(new Control100(1, 2), new Model100(-1, -2)), 0.001);
        // dv=-5.464
        assertEquals(2.732, p2.t1IminusGplus(new Control100(2, 2), new Model100(-2, -2)), 0.001);
    }

    @Test
    void testTb() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);
        // dv=5.464
        assertEquals(2.732, p2.t1IplusGminus(new Control100(-2, -2), new Model100(2, 2)), 0.001);
        // dv=4.828
        assertEquals(2.414, p2.t1IplusGminus(new Control100(-1, -2), new Model100(1, 2)), 0.001);
        // dv=4.449
        assertEquals(2.225, p2.t1IplusGminus(new Control100(-0.5, -2), new Model100(0.5, 2)), 0.001);
        // switches at G
        assertEquals(2.000, p2.t1IplusGminus(new Control100(0, -2), new Model100(0, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.t1IplusGminus(new Control100(0.5, -2), new Model100(-0.5, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.t1IplusGminus(new Control100(1, -2), new Model100(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.t1IplusGminus(new Control100(2, -2), new Model100(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.t1IminusGplus(new Control100(-2, -2), new Model100(2, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.t1IminusGplus(new Control100(-1, -2), new Model100(1, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.t1IminusGplus(new Control100(-0.5, -2), new Model100(0.5, -2)), 0.001);
        // switches at I, dv=0
        assertEquals(0.000, p2.t1IminusGplus(new Control100(0, -2), new Model100(0, 2)), 0.001);
        // dv=-0.449
        assertEquals(0.225, p2.t1IminusGplus(new Control100(0.5, -2), new Model100(-0.5, 2)), 0.001);
        // dv=-0.828
        assertEquals(0.414, p2.t1IminusGplus(new Control100(1, -2), new Model100(-1, 2)), 0.001);
        // dv=-1.464
        assertEquals(0.732, p2.t1IminusGplus(new Control100(2, -2), new Model100(-2, 2)), 0.001);
    }

    /** Verify the time to the switching point */
    @Test
    void testT1() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);
        assertEquals(0.732, p2.t1(new Control100(-2, 2), new Model100(2, 2)), 0.001);
        assertEquals(0.414, p2.t1(new Control100(-1, 2), new Model100(1, 2)), 0.001);
        assertEquals(0.225, p2.t1(new Control100(-0.5, 2), new Model100(0.5, 2)), 0.001);
        assertEquals(0.000, p2.t1(new Control100(0, 2), new Model100(0, 2)), 0.001);
        assertEquals(2.225, p2.t1(new Control100(0.5, 2), new Model100(-0.5, 2)), 0.001);
        assertEquals(2.414, p2.t1(new Control100(1, 2), new Model100(-1, 2)), 0.001);
        assertEquals(2.732, p2.t1(new Control100(2, 2), new Model100(-2, 2)), 0.001);

        assertEquals(0.732, p2.t1(new Control100(-2, 2), new Model100(2, -2)), 0.001);
        assertEquals(0.414, p2.t1(new Control100(-1, 2), new Model100(1, -2)), 0.001);
        assertEquals(0.225, p2.t1(new Control100(-0.5, 2), new Model100(0.5, -2)), 0.001);
        assertEquals(0.000, p2.t1(new Control100(0, 2), new Model100(0, -2)), 0.001);
        assertEquals(2.225, p2.t1(new Control100(0.5, 2), new Model100(-0.5, -2)), 0.001);
        assertEquals(2.414, p2.t1(new Control100(1, 2), new Model100(-1, -2)), 0.001);
        assertEquals(2.732, p2.t1(new Control100(2, 2), new Model100(-2, -2)), 0.001);

        assertEquals(2.732, p2.t1(new Control100(-2, -2), new Model100(2, 2)), 0.001);
        assertEquals(2.414, p2.t1(new Control100(-1, -2), new Model100(1, 2)), 0.001);
        assertEquals(2.225, p2.t1(new Control100(-0.5, -2), new Model100(0.5, 2)), 0.001);
        assertEquals(0.000, p2.t1(new Control100(0, -2), new Model100(0, 2)), 0.001);
        assertEquals(0.225, p2.t1(new Control100(0.5, -2), new Model100(-0.5, 2)), 0.001);
        assertEquals(0.414, p2.t1(new Control100(1, -2), new Model100(-1, 2)), 0.001);
        assertEquals(0.732, p2.t1(new Control100(2, -2), new Model100(-2, 2)), 0.001);
    }

    /** Verify paths taken */
    @Test
    void testCalculate() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);
        assertEquals(-1.959, p2.calculate(0.02, new Control100(-2, 2), new Model100(2, 2)).x(), 0.001);
        assertEquals(-0.959, p2.calculate(0.02, new Control100(-1, 2), new Model100(1, 2)).x(), 0.001);
        assertEquals(-0.459, p2.calculate(0.02, new Control100(-0.5, 2), new Model100(0.5, 2)).x(), 0.001);
        assertEquals(0.000, p2.calculate(0.02, new Control100(0, 2), new Model100(0, 2)).x(), 0.001);
        assertEquals(0.539, p2.calculate(0.02, new Control100(0.5, 2), new Model100(-0.5, 2)).x(), 0.001);
        assertEquals(1.039, p2.calculate(0.02, new Control100(1, 2), new Model100(-1, 2)).x(), 0.001);
        assertEquals(2.039, p2.calculate(0.02, new Control100(2, 2), new Model100(-2, 2)).x(), 0.001);

        assertEquals(2.04, p2.calculate(0.02, new Control100(-2, 2), new Model100(2, 2)).v(), 0.001);
        assertEquals(2.04, p2.calculate(0.02, new Control100(-1, 2), new Model100(1, 2)).v(), 0.001);
        assertEquals(2.04, p2.calculate(0.02, new Control100(-0.5, 2), new Model100(0.5, 2)).v(), 0.001);
        assertEquals(2.0, p2.calculate(0.02, new Control100(0, 2), new Model100(0, 2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Control100(0.5, 2), new Model100(-0.5, 2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Control100(1, 2), new Model100(-1, 2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Control100(2, 2), new Model100(-2, 2)).v(), 0.001);

    }

    @Test
    void testCalculateA() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);
        assertEquals(-1.959, p2.calculate(0.02, new Control100(-2, 2), new Model100(2, -2)).x(), 0.001);
        assertEquals(-0.959, p2.calculate(0.02, new Control100(-1, 2), new Model100(1, -2)).x(), 0.001);
        assertEquals(-0.459, p2.calculate(0.02, new Control100(-0.5, 2), new Model100(0.5, -2)).x(), 0.001);
        assertEquals(0.039, p2.calculate(0.02, new Control100(0, 2), new Model100(0, -2)).x(), 0.001);
        assertEquals(0.539, p2.calculate(0.02, new Control100(0.5, 2), new Model100(-0.5, -2)).x(), 0.001);
        assertEquals(1.039, p2.calculate(0.02, new Control100(1, 2), new Model100(-1, -2)).x(), 0.001);
        assertEquals(2.039, p2.calculate(0.02, new Control100(2, 2), new Model100(-2, -2)).x(), 0.001);

        assertEquals(2.04, p2.calculate(0.02, new Control100(-2, 2), new Model100(2, -2)).v(), 0.001);
        assertEquals(2.04, p2.calculate(0.02, new Control100(-1, 2), new Model100(1, -2)).v(), 0.001);
        assertEquals(2.04, p2.calculate(0.02, new Control100(-0.5, 2), new Model100(0.5, -2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Control100(0, 2), new Model100(0, -2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Control100(0.5, 2), new Model100(-0.5, -2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Control100(1, 2), new Model100(-1, -2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Control100(2, 2), new Model100(-2, -2)).v(), 0.001);
    }

    @Test
    void testCalculateB() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);
        assertEquals(-2.039, p2.calculate(0.02, new Control100(-2, -2), new Model100(2, 2)).x(), 0.001);
        assertEquals(-1.039, p2.calculate(0.02, new Control100(-1, -2), new Model100(1, 2)).x(), 0.001);
        assertEquals(-0.539, p2.calculate(0.02, new Control100(-0.5, -2), new Model100(0.5, 2)).x(), 0.001);
        assertEquals(-0.039, p2.calculate(0.02, new Control100(0, -2), new Model100(0, 2)).x(), 0.001);
        assertEquals(0.459, p2.calculate(0.02, new Control100(0.5, -2), new Model100(-0.5, 2)).x(), 0.001);
        assertEquals(0.959, p2.calculate(0.02, new Control100(1, -2), new Model100(-1, 2)).x(), 0.001);
        assertEquals(1.959, p2.calculate(0.02, new Control100(2, -2), new Model100(-2, 2)).x(), 0.001);

        assertEquals(-1.96, p2.calculate(0.02, new Control100(-2, -2), new Model100(2, 2)).v(), 0.001);
        assertEquals(-1.96, p2.calculate(0.02, new Control100(-1, -2), new Model100(1, 2)).v(), 0.001);
        assertEquals(-1.96, p2.calculate(0.02, new Control100(-0.5, -2), new Model100(0.5, 2)).v(), 0.001);
        assertEquals(-1.96, p2.calculate(0.02, new Control100(0, -2), new Model100(0, 2)).v(), 0.001);
        assertEquals(-2.04, p2.calculate(0.02, new Control100(0.5, -2), new Model100(-0.5, 2)).v(), 0.001);
        assertEquals(-2.04, p2.calculate(0.02, new Control100(1, -2), new Model100(-1, 2)).v(), 0.001);
        assertEquals(-2.04, p2.calculate(0.02, new Control100(2, -2), new Model100(-2, 2)).v(), 0.001);
    }

    @Test
    void testSwitchingTime() {
        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);
        // between (-2,2) and (2,2) the switching point is at (0, 3.464)
        // at the switching point,
        // u=-2, v=3.464, dt=0.02, dx = 0.0693 + 0.0004, dv=0.04

        // 0.02s before the switching point should yield the switching point exactly
        Control100 s = p2.calculate(0.02, new Control100(-0.0693, 3.424), new Model100(2, 2));
        assertEquals(0.000, s.x(), 0.001);
        assertEquals(3.464, s.v(), 0.001);

        // this is right at the switching point: the correct path is 0.02 down G-
        s = p2.calculate(0.02, new Control100(0, 3.464), new Model100(2, 2));
        assertEquals(0.0693, s.x(), 0.001);
        assertEquals(3.424, s.v(), 0.001);

        // split dt between I+ and G-
        // u=-2, v=3.464, dt=0.01, dx = 0.0346 + 0.0001, dv=0.02
        // the correct outcome is 0.01 down G-
        s = p2.calculate(0.02, new Control100(-0.0346, 3.444), new Model100(2, 2));
        assertEquals(0.0346, s.x(), 0.001);
        assertEquals(3.444, s.v(), 0.001);
    }

    @Test
    void testQDotSwitch() {
        TrapezoidIncrementalProfile p = new TrapezoidIncrementalProfile(logger.name("one"), 5, 1, 0.01);

        TrapezoidIncrementalProfile p2 = new TrapezoidIncrementalProfile(logger.name("two"), 5, 2, 0.01);

        assertEquals(1.224, p2.qDotSwitchIplusGminus(new Control100(0, 0), new Model100(0.5, 1.0)), 0.001);
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Control100(0, 0), new Model100(0.5, 1.0)), 0.001);

        assertEquals(3.000, p.qDotSwitchIplusGminus(new Control100(-3, 2), new Model100(2, 2)), 0.001);
        assertEquals(2.828, p.qDotSwitchIplusGminus(new Control100(-2, 2), new Model100(2, 2)), 0.001);
        assertEquals(2.645, p.qDotSwitchIplusGminus(new Control100(-1, 2), new Model100(2, 2)), 0.001);

        assertEquals(-3.0, p.qDotSwitchIminusGplus(new Control100(2, -2), new Model100(-3, -2)), 0.001);
        assertEquals(-2.828, p.qDotSwitchIminusGplus(new Control100(2, -2), new Model100(-2, -2)), 0.001);
        assertEquals(-2.645, p.qDotSwitchIminusGplus(new Control100(2, -2), new Model100(-1, -2)), 0.001);

        // from 2,2 to -2,2. There's no intersection between these curves
        assertEquals(Double.NaN, p.qDotSwitchIplusGminus(new Control100(2, 2), new Model100(-2, 2)), 0.001);
        // from -2,2 to 2,-2 switches in the same place as -2,2->2,2
        assertEquals(2.828, p.qDotSwitchIplusGminus(new Control100(-2, 2), new Model100(2, -2)), 0.001);
        // from 2,2 to -2,2 switches at the bottom
        assertEquals(-2.828, p.qDotSwitchIminusGplus(new Control100(2, 2), new Model100(-2, 2)), 0.001);
        // from -2,2 to 2,-2, I-G+ is invalid
        assertEquals(Double.NaN, p.qDotSwitchIminusGplus(new Control100(-2, 2), new Model100(2, -2)), 0.001);

    }

    /**
     * this is a normal profile from 0 to 1, rest-to-rest, it's a triangle profile.
     */
    @Test
    void testTriangle() {
        TrapezoidIncrementalProfile profileX = new TrapezoidIncrementalProfile(logger, 5, 2, 0.1);
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

    /**
     * this is an inverted profile from 0 to -1, rest-to-rest, it's a triangle
     * profile, it's exactly the inverse of the case above.
     */
    @Test
    void testInvertedTriangle() {
        TrapezoidIncrementalProfile profileX = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(-1, 0);

        // the first sample is near the starting state
        dump(0, sample);
        sample = profileX.calculate(0.02, sample, end);
        assertEquals(0, sample.x(), DELTA);
        assertEquals(-0.04, sample.v(), DELTA);

        // step to the middle of the profile
        for (double t = 0; t < 0.68; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            dump(t, sample);
        }
        // halfway there, going fast
        assertEquals(-0.5, sample.x(), 0.01);
        assertEquals(-1.4, sample.v(), DELTA);

        // step to the end of the profile ... this was 0.72.
        for (double t = 0; t < 0.86; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            dump(t, sample);
        }
        assertEquals(-1.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /** with a lower top speed, this profile includes a cruise phase. */
    @Test
    void testCruise() {
        TrapezoidIncrementalProfile profileX = new TrapezoidIncrementalProfile(logger, 1, 2, 0.01);
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

        // step to the cruise phase of the profile
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.25, sample.x(), 0.01);
        assertEquals(1.0, sample.v(), DELTA);

        // step to near the end of cruise
        for (double t = 0; t < 0.5; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.75, sample.x(), 0.01);
        assertEquals(1.00, sample.v(), DELTA);

        // step to the end of the profile // this used to be 0.5
        for (double t = 0; t < 0.66; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(1.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /**
     * this is a "u-turn" profile, initially heading away from the goal.
     * overshoot works correctly.
     */
    @Test
    void testUTurn() {
        TrapezoidIncrementalProfile profileX = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);

        // initially heading away from the goal
        Control100 sample = new Control100(0.1, 1);
        final Model100 end = new Model100(0, 0);

        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);
        assertEquals(0.120, sample.x(), DELTA);
        assertEquals(0.96, sample.v(), DELTA);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.35, sample.x(), DELTA);
        assertEquals(0.0, sample.v(), DELTA);

        // the next phase is triangular, this is the point at maximum speed
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.19, sample.x(), DELTA);
        assertEquals(-0.8, sample.v(), DELTA);

        // this is the end. this was 0.44
        for (double t = 0; t < 0.46; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /** Same as above but not inverted. */
    @Test
    void testUTurnNotInverted() {
        TrapezoidIncrementalProfile profileX = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);

        // initially heading away from the goal
        Control100 sample = new Control100(-0.1, -1, 0);
        final Model100 end = new Model100(0, 0);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);

        assertEquals(-0.120, sample.x(), DELTA);
        assertEquals(-0.96, sample.v(), DELTA);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(-0.35, sample.x(), DELTA);
        assertEquals(0.0, sample.v(), DELTA);

        // the next phase is triangular, this is the point at maximum speed
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(-0.19, sample.x(), DELTA);
        assertEquals(0.8, sample.v(), DELTA);

        // this is the end. this was 0.44.
        for (double t = 0; t < 0.46; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /**
     * the same logic should work if the starting position is *at* the goal.
     * 
     * the WPI profile fails this test, but the bangbang controller passes.
     */
    @Test
    void testUTurn2() {
        TrapezoidIncrementalProfile profileX = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);

        // initially at the goal with nonzero velocity
        Control100 sample = new Control100(0, 1);
        final Model100 end = new Model100(0, 0);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);

        assertEquals(0.02, sample.x(), DELTA);
        assertEquals(0.96, sample.v(), DELTA);

        // step to the turn-around point
        // this takes the same time no matter the starting point.
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);

        }
        assertEquals(0.25, sample.x(), DELTA);
        assertEquals(0.0, sample.v(), DELTA);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);

        }
        assertEquals(0.1, sample.x(), 0.01);
        assertEquals(-0.615, sample.v(), 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);

        }
        assertEquals(0.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /**
     * the same logic should work if the starting position is *at* the goal.
     */
    @Test
    void testUTurn2NotInverted() {
        TrapezoidIncrementalProfile profileX = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);

        // initially at the goal with nonzero velocity
        Control100 sample = new Control100(0, -1);
        final Model100 end = new Model100(0, 0);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);
        assertEquals(-0.02, sample.x(), DELTA);
        assertEquals(-0.96, sample.v(), DELTA);

        // step to the turn-around point
        // this takes the same time no matter the starting point.
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(-0.25, sample.x(), DELTA);
        assertEquals(0.0, sample.v(), DELTA);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(-0.1, sample.x(), 0.01);
        assertEquals(0.615, sample.v(), 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.44; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /**
     * And it should work if the starting position is to the *left* of the goal, too
     * fast to stop.
     * 
     * The WPI trapezoid profile fails this test, but the bangbang controller
     * passes.
     */
    @Test
    void testUTurn3() {
        TrapezoidIncrementalProfile profileX = new TrapezoidIncrementalProfile(logger, 5, 2, 0.01);

        // behind the goal, too fast to stop.
        Control100 sample = new Control100(-0.1, 1);
        final Model100 end = new Model100(0, 0);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);
        assertEquals(-0.08, sample.x(), DELTA);
        assertEquals(0.96, sample.v(), DELTA);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.15, sample.x(), DELTA);
        assertEquals(0.0, sample.v(), DELTA);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.26; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.07, sample.x(), 0.01);
        assertEquals(-0.53, sample.v(), 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.6; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    @Test
    void testWindupCase() {
        TrapezoidIncrementalProfile profileX = new TrapezoidIncrementalProfile(logger, 5, 2, 0.05);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(0, 1);
        sample = profileX.calculate(0.02, sample, end);
        // I- means dv = 2 * 0.02 = 0.04 and dx = 0.0004
        assertEquals(-0.0004, sample.x(), 0.000001);
        assertEquals(-0.04, sample.v(), 0.000001);
        sample = profileX.calculate(0.02, sample, end);
        // still I-, dv = 0.04 more, dx = 0.0004 + 0.0008 + 0.0004
        assertEquals(-0.0016, sample.x(), 0.000001);
        assertEquals(-0.08, sample.v(), 0.000001);
    }

    /**
     * initially at rest, we want a state in the same position but moving, so this
     * requires a "windup" u-turn.
     * 
     * The WPI profile fails this test, but the bangbang controller passes.
     */
    @Test
    void testUTurnWindup() {
        TrapezoidIncrementalProfile profileX = new TrapezoidIncrementalProfile(logger, 5, 2, 0.05);

        // initially at rest
        Control100 sample = new Control100(0, 0);
        // goal is moving
        final Model100 end = new Model100(0, 1);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);

        assertEquals(0, sample.x(), DELTA);
        assertEquals(-0.04, sample.v(), DELTA);

        // step to the turn-around point
        for (double t = 0; t < 0.7; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
            if (sample.model().near(end, 0.05))
                break;

            // if (profileX.isFinished())
            // break;
        }
        assertEquals(-0.25, sample.x(), 0.01);
        assertEquals(0.02, sample.v(), 0.01);

        for (double t = 0; t < 1; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
            if (sample.model().near(end, 0.05))
                break;
            // if (profileX.isFinished())
            // break;

        }
        assertEquals(0, sample.x(), 0.05);
        assertEquals(1, sample.v(), 0.05);

    }

    //////////////////////////////////////////////////////

    // Tests below are from WPILib TrapezoidProfileTest.

    /**
     * Asserts "val1" is less than or equal to "val2".
     *
     * @param val1 First operand in comparison.
     * @param val2 Second operand in comparison.
     */
    private static void assertLessThanOrEquals(double val1, double val2) {
        assertTrue(val1 <= val2, val1 + " is greater than " + val2);
    }

    private static void assertNear(double val1, double val2, double eps) {
        assertEquals(val1, val2, eps);
    }

    /**
     * Asserts "val1" is less than or within "eps" of "val2".
     *
     * @param val1 First operand in comparison.
     * @param val2 Second operand in comparison.
     * @param eps  Tolerance for whether values are near to each other.
     */
    private static void assertLessThanOrNear(double val1, double val2, double eps) {
        if (val1 <= val2) {
            assertLessThanOrEquals(val1, val2);
        } else {
            assertNear(val1, val2, eps);
        }
    }

    @Test
    void reachesGoal() {
        final Model100 goal = new Model100(3, 0);
        Control100 state = new Control100(0, 0);

        TrapezoidIncrementalProfile profile = new TrapezoidIncrementalProfile(logger, 1.75, 0.75, 0.01);
        for (int i = 0; i < 450; ++i) {
            state = profile.calculate(TEN_MS, state, goal);
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

    // Tests that decreasing the maximum velocity in the middle when it is already
    // moving faster than the new max is handled correctly
    // Oct 20, 2024, this behavior is different now. It used to
    // immediately clamp the profile to the new maximum, with
    // infinite acceleration, which is a pointless behavior. Now
    // the new constraint creates max braking.
    @Test
    void posContinuousUnderVelChange() {
        Model100 goal = new Model100(12, 0);

        TrapezoidIncrementalProfile profile = new TrapezoidIncrementalProfile(logger.name("one"), 1.75, 0.75, 0.01);
        Control100 state = profile.calculate(TEN_MS, new Control100(0, 0), goal);

        double lastPos = state.x();
        for (int i = 0; i < 1600; ++i) {
            if (i == 400) {
                // impose new slower limit
                profile = new TrapezoidIncrementalProfile(logger.name("two"), 0.75, 0.75, 0.01);
            }

            state = profile.calculate(TEN_MS, state, goal);
            double estimatedVel = (state.x() - lastPos) / TEN_MS;

            // wait 1.35 sec to brake to the new max velocity
            if (i >= 535) {
                // Since estimatedVel can have floating point rounding errors, we check
                // whether value is less than or within an error delta of the new
                // constraint.
                assertLessThanOrNear(estimatedVel, 0.75, 0.01);

                assertLessThanOrEquals(state.v(), 0.75);
            }

            lastPos = state.x();
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

    // There is some somewhat tricky code for dealing with going backwards
    @Test
    void backwards() {
        final Model100 goal = new Model100(-2, 0);
        Control100 state = new Control100(0, 0);

        TrapezoidIncrementalProfile profile = new TrapezoidIncrementalProfile(logger, 0.75, 0.75, 0.01);
        for (int i = 0; i < 400; ++i) {
            state = profile.calculate(TEN_MS, state, goal);
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

    @Test
    void switchGoalInMiddle() {
        Model100 goal = new Model100(-2, 0);
        Control100 state = new Control100(0, 0);

        TrapezoidIncrementalProfile profile = new TrapezoidIncrementalProfile(logger, 0.75, 0.75, 0.01);
        for (int i = 0; i < 200; ++i) {
            state = profile.calculate(TEN_MS, state, goal);
        }
        assertNotEquals(state, goal);

        goal = new Model100(0.0, 0.0);
        profile = new TrapezoidIncrementalProfile(logger, 0.75, 0.75, 0.01);
        for (int i = 0; i < 600; ++i) {
            state = profile.calculate(TEN_MS, state, goal);
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

    // Checks to make sure that it hits top speed
    @Test
    void topSpeed() {
        final Model100 goal = new Model100(4, 0);
        Control100 state = new Control100(0, 0);

        TrapezoidIncrementalProfile profile = new TrapezoidIncrementalProfile(logger, 0.75, 0.75, 0.01);
        for (int i = 0; i < 200; ++i) {
            state = profile.calculate(TEN_MS, state, goal);
        }
        assertNear(0.75, state.v(), 10e-5);

        profile = new TrapezoidIncrementalProfile(logger, 0.75, 0.75, 0.01);
        for (int i = 0; i < 2000; ++i) {
            state = profile.calculate(TEN_MS, state, goal);
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

}
