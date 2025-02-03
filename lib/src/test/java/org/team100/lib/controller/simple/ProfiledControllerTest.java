package org.team100.lib.controller.simple;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.Profile100.ResultWithETA;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

public class ProfiledControllerTest {
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    private static final double kDelta = 0.001;

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

    /**
     * I think we have a habit of mixing up previous-step, current-step, and
     * future-step quantities when writing profile/control loops. This verifies the
     * right way to do it.
     */
    @Test
    void test1() {

    }

    /** I think we're writing followers incorrectly, here's how to do it. */
    @Test
    void discreteTime1() {
        final Profile100 profile = new TrapezoidProfile100(2, 1, 0.01);
        final Model100 initial = new Model100(0, 0);
        final Model100 goal = new Model100(1, 0);
        final double k1 = 5.0;
        final double k2 = 1.0;

        FullStateFeedback controller = new FullStateFeedback(
                logger, k1, k2, x -> x, 1, 1);

        // initial state is motionless
        Sim sim = new Sim();
        sim.y = 0;
        sim.yDot = 0;
        // double feedback = 0;
        Control100 feedback = new Control100();
        Control100 setpointControl = new Control100();

        Model100 setpointModel = initial;
        Util.printf(" t,      x,      v,      a,      y,      ydot,  fb,   eta\n");

        // log initial state
        Util.printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                0.0, setpointModel.x(), setpointModel.v(), 0.0, sim.y, sim.yDot, 0.0, 0.0);

        // eta to goal
        double etaS = 0;
        for (double currentTime = 0.0; currentTime < 3; currentTime += 0.02) {

            // at the beginning of the time step, we show the current measurement
            // and the setpoint calculated in the previous time step (which applies to this
            // one)
            Util.printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                    currentTime,
                    setpointControl.x(),
                    setpointControl.v(),
                    setpointControl.a(),
                    sim.y,
                    sim.yDot,
                    feedback.v(),
                    etaS);

            // compute feedback using the "previous" setpoint, which is for the current
            // instant

            feedback = controller.calculate(new Model100(sim.y, sim.yDot), setpointModel);

            ResultWithETA result = profile.calculateWithETA(0.02, setpointModel, goal);
            setpointControl = result.state();
            etaS = result.etaS();
            // this is the setpoint for the next time step
            setpointModel = setpointControl.model();

            // this is actuation for the next time step, using the feedback for the current
            // time, and feedforward for the next time step

            // TODO: fix this combination of "a" and feedback "v"
            double u = setpointControl.a() + feedback.v();

            sim.step(u);
        }
    }
}
