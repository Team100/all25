package org.team100.lib.controller.simple;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.Profile100.ResultWithETA;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * A profiled controller combines a source of references (e.g. a trapezoid, or
 * any other method) and a source of feedback (e.g. PID, or any other method).
 * 
 * I think we have a habit of mixing up previous-step, current-step, and
 * future-step quantities when writing profile/control loops. This implements
 * the right way to do it.
 */
public class ProfiledController implements Controller100 {
    private static final double kDt = TimedRobot100.LOOP_PERIOD_S;
    private final Profile100 m_profile;
    private final Feedback100 m_feedback;
    private Model100 setpointModel;

    public ProfiledController(Profile100 profile, Feedback100 feedback) {
        m_profile = profile;
        m_feedback = feedback;
    }

    public void init(Model100 initial) {
        setpointModel = initial;
    }

    @Override
    public Control100 calculate(Model100 measurement, Model100 goal) {
        if (setpointModel == null)
            throw new IllegalStateException("must initialize");
        // there are two components: profile and feedback.
        // feedback compares the current measurement with the current profile,
        // and applies the output to the next time step.
        // profile is for the next time step.
        double u_FB = m_feedback.calculate(measurement, setpointModel);
        ResultWithETA result = m_profile.calculateWithETA(kDt, setpointModel, goal);
        Control100 setpointControl = result.state();
        setpointModel = setpointControl.model();

        // there's more than one way to apply the feedback.
        // sometimes we apply it as velocity, so a lower level servo uses it.
        // for now it's accel.
        double a = setpointControl.a() + u_FB;
        double v = measurement.v() + a * kDt;
        double x = measurement.x() + measurement.v() * kDt + 0.5 * a * kDt * kDt;
        return new Control100(x, v, a);
    }

    @Override
    public boolean atSetpoint() {
        return false;
    }

    @Override
    public void reset() {
        // nothing
    }

}
