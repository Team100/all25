package org.team100.lib.controller.simple;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.profile.timed.JerkLimitedProfile100;
import org.team100.lib.profile.timed.SepticSplineProfile;
import org.team100.lib.profile.timed.TimedProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

public class TimedProfiledControllerTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSpline() {
        TimedProfile wristProfile = new SepticSplineProfile(35, 15);
        Feedback100 wristFeedback = new PIDFeedback(
                logger, 7.5, 0.00, 0.000, false, 0.01, 0.01);
        ProfiledController c = new TimedProfiledController(logger,
                wristProfile, wristFeedback, x -> x, 0.01, 0.01);

        Model100 setpoint = new Model100();
        c.init(setpoint);
        final Model100 goal = new Model100(1, 0);

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
        TimedProfile wristProfile = new JerkLimitedProfile100(35, 15, 60, false);
        Feedback100 wristFeedback = new PIDFeedback(
                logger, 7.5, 0.00, 0.000, false, 0.01, 0.01);
        ProfiledController c = new TimedProfiledController(logger,
                wristProfile, wristFeedback, x -> x, 0.01, 0.01);

        Model100 setpoint = new Model100();
        c.init(setpoint);
        final Model100 goal = new Model100(1, 0);

        for (int i = 0; i < 100; ++i) {
            stepTime();
            ProfiledController.Result result = c.calculate(setpoint, goal);
            Control100 ff = result.feedforward();
            setpoint = ff.model();
            if (DEBUG)
                Util.printf("%12.3f %12.3f%12.3f%12.3f\n", Takt.get(), ff.x(), ff.v(), ff.a());
        }
    }
}
