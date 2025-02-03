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
 * 
 * You must call init() with the current measurement, prior to calculate().
 * 
 * Remember to re-initialize anytime the profile may diverge from the
 * measurement, or you'll introduce transients.
 */
public class ProfiledController {

    /**
     * @param feedforward the desired state for the end of the next time step.
     * @param feedback    based on the error at the current instant.
     */
    public record Result(Control100 feedforward, double feedback) {
    }

    private static final double kDt = TimedRobot100.LOOP_PERIOD_S;
    private final Profile100 m_profile;
    private final Feedback100 m_feedback;
    private Model100 m_setpoint;

    public ProfiledController(Profile100 profile, Feedback100 feedback) {
        m_profile = profile;
        m_feedback = feedback;
    }

    /**
     * Initializes the setpoint.
     * 
     * @param measurement current-instant measurement
     */
    public void init(Model100 measurement) {
        m_setpoint = measurement;
        m_feedback.reset();
    }

    /**
     * Calculates feedforward and feedback.
     * 
     * @param measurement current-instant measurement
     * @param goal        final desired state
     */
    public Result calculate(Model100 measurement, Model100 goal) {
        if (m_setpoint == null)
            throw new IllegalStateException("Null setpoint!");

        // Feedback compares the current measurement with the current profile.
        double u_FB = m_feedback.calculate(measurement, m_setpoint);

        // Profile result is for the next time step.
        ResultWithETA result = m_profile.calculateWithETA(kDt, m_setpoint, goal);

        Control100 u_FF = result.state();
        m_setpoint = u_FF.model();

        return new Result(u_FF, u_FB);
    }
}
