package org.team100.lib.controller.simple;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.profile.timed.JerkLimitedProfile100;
import org.team100.lib.profile.timed.SepticSplineProfile;
import org.team100.lib.profile.timed.TimedProfile;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.reference.TimedProfileReference1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

public class TimedProfiledControllerTest implements Timeless {
    private static final double kDelta = 1e-12;
    private static final boolean DEBUG = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSpline() {
        final Model100 goal = new Model100(1, 0);
        TimedProfile wristProfile = new SepticSplineProfile(35, 15);
        TimedProfileReference1d ref = new TimedProfileReference1d(wristProfile, goal);

        Feedback100 wristFeedback = new PIDFeedback(
                logger, 7.5, 0.00, 0.000, false, 0.01, 0.01);
        ProfiledController c = new TimedProfiledController(logger,
                ref, wristFeedback, x -> x, 0.01, 0.01);

        Model100 setpoint = new Model100();
        c.init(setpoint);

        for (int i = 0; i < 100; ++i) {
            stepTime();
            ProfiledController.Result result = c.calculate(setpoint, goal);
            Control100 ff = result.feedforward();
            setpoint = ff.model();
            if (DEBUG)
                Util.printf("%12.3f %12.3f%12.3f%12.3f\n", Takt.get(), ff.x(), ff.v(), ff.a());
        }
    }

    @Test
    void testJerkLimited() {
        final Model100 goal = new Model100(1, 0);
        TimedProfile wristProfile = new JerkLimitedProfile100(35, 15, 60, false);
        TimedProfileReference1d ref = new TimedProfileReference1d(wristProfile, goal);
        Feedback100 wristFeedback = new PIDFeedback(
                logger, 7.5, 0.00, 0.000, false, 0.01, 0.01);
        ProfiledController c = new TimedProfiledController(logger,
                ref, wristFeedback, x -> x, 0.01, 0.01);

        Model100 setpoint = new Model100();
        c.init(setpoint);

        for (int i = 0; i < 100; ++i) {
            stepTime();
            Setpoints1d setpoints = ref.get();
            setpoint = setpoints.next().model();
            if (DEBUG)
                Util.printf("%12.3f %12.3f%12.3f%12.3f\n", Takt.get(),
                 setpoints.next().x(), setpoints.next().v(), setpoints.next().a());
        }
    }

    /** This covers refactoring the controller */
    @Test
    void actuallyTestSomething() {
        Model100 goal = new Model100(1, 0);
        TimedProfile p = new JerkLimitedProfile100(2, 6, 25, false);
        TimedProfileReference1d ref = new TimedProfileReference1d(p, goal);
        Feedback100 fb = new PositionProportionalFeedback(1, 0.01);
        ProfiledController c = new TimedProfiledController(logger, ref, fb, x -> x, 0.01, 0.01);
        Model100 setpoint = new Model100();
        c.init(setpoint);
        ProfiledController.Result result = c.calculate(setpoint, goal);
        assertEquals(0, result.feedback(), kDelta);
        assertEquals(0.0000333333333, result.feedforward().x(), kDelta);
        assertEquals(0.005, result.feedforward().v(), kDelta);
        assertEquals(0.5, result.feedforward().a(), kDelta);

        setpoint = result.feedforward().model();

        // no time step => no change
        result = c.calculate(setpoint, goal);
        assertEquals(0, result.feedback(), kDelta);
        assertEquals(0.000033333333333333, result.feedforward().x(), kDelta);
        assertEquals(0.005, result.feedforward().v(), kDelta);
        assertEquals(0.5, result.feedforward().a(), kDelta);

        setpoint = result.feedforward().model();

        stepTime();

        // now we've advanced.
        result = c.calculate(setpoint, goal);
        assertEquals(0, result.feedback(), kDelta);
        assertEquals(0.000266666666666666, result.feedforward().x(), kDelta);
        assertEquals(0.020, result.feedforward().v(), kDelta);
        assertEquals(1.000, result.feedforward().a(), kDelta);

    }
}
