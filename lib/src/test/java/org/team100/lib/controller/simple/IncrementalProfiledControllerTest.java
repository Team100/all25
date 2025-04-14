package org.team100.lib.controller.simple;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.Profile100.ResultWithETA;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.profile.incremental.TrapezoidProfileWPI;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Util;

public class IncrementalProfiledControllerTest implements Timeless {
    private static final double kDelta = 0.001;
    private static final boolean DEBUG = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    /** Double integrator system simulator, kinda */
    static class Sim {
        /** this represents the system's inability to execute infinite jerk. */
        private final double m_jerkLimit;
        /** measured position */
        double y = 0;
        /** measured velocity */
        double yDot = 0;
        /** accel for imposing the jerk limit */
        double a = 0;

        public Sim(double jerkLimit) {
            m_jerkLimit = jerkLimit;
        }

        /** evolve the system over the duration of this time step */
        void step(double u) {

            a = m_jerkLimit * a + (1 - m_jerkLimit) * u;

            y = y + yDot * 0.02 + 0.5 * a * 0.02 * 0.02;
            yDot = yDot + a * 0.02;
        }

        Model100 state() {
            return new Model100(y, yDot);
        }
    }

    @Test
    void test2() {
        Model100 goal = new Model100(0.1, 0);
        Profile100 p = new TrapezoidProfile100(100, 100, 0.01);
        IncrementalProfileReference1d ref = new IncrementalProfileReference1d(p, goal);
        // Profile100 p = new ProfileWPI(100, 100);
        final double k1 = 5.0;
        final double k2 = 1.0;
        Feedback100 f = new FullStateFeedback(logger, k1, k2, x -> x, 1, 1);

        ProfiledController controller = new IncrementalProfiledController(
                logger, ref, f, x -> x, 0.05, 0.05);
        Model100 measurement = new Model100(0, 0);
        controller.init(measurement);
        ProfiledController.Result result = controller.calculate(measurement, goal);
        // this is for the *next* timestep so there should be non-zero velocity.
        assertEquals(2, result.feedforward().v(), 1e-12);

    }

    /**
     * I think we have a habit of mixing up previous-step, current-step, and
     * future-step quantities when writing profile/control loops. This verifies the
     * right way to do it.
     */
    @Test
    void test1() {
        final Model100 goal = new Model100(1, 0);
        Profile100 p = new TrapezoidProfile100(2, 1, 0.01);
        IncrementalProfileReference1d ref = new IncrementalProfileReference1d(p, goal);

        // Feedback100 f = new PIDFeedback(logger, 1, 0, 0, false, 0.05, 1);

        final double k1 = 5.0;
        final double k2 = 1.0;
        Feedback100 f = new FullStateFeedback(logger, k1, k2, x -> x, 1, 1);

        ProfiledController c = new IncrementalProfiledController(
                logger, ref, f, x -> x, 0.05, 0.05);
        final Model100 initial = new Model100(0, 0);

        c.init(initial);

        Control100 setpointControl = initial.control();

        // for now, infinite jerk
        Sim sim = new Sim(0);
        // Sim sim = new Sim(0.9);
        sim.y = 0;
        sim.yDot = 0;
        double u_FB = 0;
        if (DEBUG)
            Util.printf(" t,      x,      v,      a,      y,      ydot,  fb\n");

        for (double currentTime = 0.0; currentTime < 3; currentTime += 0.02) {
            // at the beginning of the time step, we show the current measurement
            // and the setpoint calculated in the previous time step (which applies to this
            // one)
            if (DEBUG)
                Util.printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                        currentTime,
                        setpointControl.x(),
                        setpointControl.v(),
                        setpointControl.a(),
                        sim.y,
                        sim.yDot,
                        u_FB);

            ProfiledController.Result result = c.calculate(sim.state(), goal);
            setpointControl = result.feedforward();
            u_FB = result.feedback();
            sim.step(setpointControl.a() + u_FB);

        }

    }

    /** I think we're writing followers incorrectly, here's how to do it. */
    @Test
    void discreteTime1() {
        final Profile100 profile = new TrapezoidProfile100(2, 1, 0.01);
        final Model100 initial = new Model100(0, 0);
        final Model100 goal = new Model100(1, 0);
        final double k1 = 5.0;
        final double k2 = 1.0;

        Feedback100 feedback = new FullStateFeedback(logger, k1, k2, x -> x, 1, 1);

        // initial state is motionless
        Sim sim = new Sim(0);
        // Sim sim = new Sim(0.9);
        sim.y = 0;
        sim.yDot = 0;
        double u_FB = 0;
        Control100 setpointControl = new Control100();

        Model100 setpointModel = initial;
        if (DEBUG)
            Util.printf(" t,      x,      v,      a,      y,      ydot,  fb\n");

        // log initial state
        if (DEBUG)
            Util.printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                    0.0, setpointModel.x(), setpointModel.v(), 0.0, sim.y, sim.yDot, 0.0);

        for (double currentTime = 0.0; currentTime < 3; currentTime += 0.02) {

            // at the beginning of the time step, we show the current measurement
            // and the setpoint calculated in the previous time step (which applies to this
            // one)
            if (DEBUG)
                Util.printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                        currentTime,
                        setpointControl.x(),
                        setpointControl.v(),
                        setpointControl.a(),
                        sim.y,
                        sim.yDot,
                        u_FB);

            // compute feedback using the "previous" setpoint, which is for the current
            // instant

            u_FB = feedback.calculate(new Model100(sim.y, sim.yDot), setpointModel);

            ResultWithETA result = profile.calculateWithETA(0.02, setpointModel.control(), goal);
            setpointControl = result.state();
            // this is the setpoint for the next time step
            setpointModel = setpointControl.model();

            // this is actuation for the next time step, using the feedback for the current
            // time, and feedforward for the next time step

            double u = setpointControl.a() + u_FB;

            sim.step(u);
        }
    }

    /** This covers refactoring the controller */
    @Test
    void actuallyTestSomething() {
        Model100 goal = new Model100(1, 0);
        TrapezoidProfileWPI p = new TrapezoidProfileWPI(2, 6);
        IncrementalProfileReference1d ref = new IncrementalProfileReference1d(p, goal);
        Feedback100 fb = new PositionProportionalFeedback(1, 0.01);
        ProfiledController c = new IncrementalProfiledController(
                logger, ref, fb, x -> x, 0.01, 0.01);
        Model100 setpoint = new Model100();
        c.init(setpoint);

        ProfiledController.Result result = c.calculate(setpoint, goal);
        assertEquals(0, result.feedback(), kDelta);
        assertEquals(0.0012, result.feedforward().x(), kDelta);
        assertEquals(0.120, result.feedforward().v(), kDelta);
        assertEquals(6.000, result.feedforward().a(), kDelta);

        setpoint = result.feedforward().model();

        // the incremental profile ignores the clock, so this advances
        result = c.calculate(setpoint, goal);
        assertEquals(0, result.feedback(), kDelta);
        assertEquals(0.0048, result.feedforward().x(), kDelta);
        assertEquals(0.240, result.feedforward().v(), kDelta);
        assertEquals(6.000, result.feedforward().a(), kDelta);

        setpoint = result.feedforward().model();

        stepTime();

        // clock is ignored, advance again.
        result = c.calculate(setpoint, goal);
        assertEquals(0, result.feedback(), kDelta);
        assertEquals(0.0108, result.feedforward().x(), kDelta);
        assertEquals(0.360, result.feedforward().v(), kDelta);
        assertEquals(6.000, result.feedforward().a(), kDelta);

    }
}
