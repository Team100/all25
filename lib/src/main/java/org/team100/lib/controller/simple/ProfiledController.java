package org.team100.lib.controller.simple;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.Profile100.ResultWithETA;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

/**
 * A profiled controller combines a source of references (e.g. a trapezoid, or
 * any other method) and a source of feedback (e.g. PID, or any other method).
 * 
 * This class remembers the setpoint. When you call calculate(), you make a new
 * setpoint intended for the end of the next time step.
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
    // TODO: make these constructor args
    private static final double kPositionTolerance = 0.05;
    private static final double kVelocityTolerance = 0.05;
    private final Profile100 m_profile;
    private final Feedback100 m_feedback;
    private final DoubleUnaryOperator m_modulus;
    private Model100 m_setpoint;

    /**
     * TODO: include modulus some other way: the profile and feedback need to both
     * also be rotary, if this controller is rotary.
     */
    public ProfiledController(
            Profile100 profile,
            Feedback100 feedback,
            DoubleUnaryOperator modulus) {
        m_profile = profile;
        m_feedback = feedback;
        m_modulus = modulus;
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
     * Feedback is based on the error between the current-instant measurement and
     * the setpoint calculated in the past, which was also intended for the current
     * instant.
     * 
     * Feedforward is based on the profile at the end of the next time step.
     * 
     * This remembers the feedforward and uses it on the next step.
     * 
     * @param measurement current-instant measurement
     * @param goal        final desired state
     */
    public Result calculate(Model100 measurement, Model100 goal) {
        // Util.printf("ProfiledController measurement %s goal %s\n", measurement, goal);
        if (m_setpoint == null)
            throw new IllegalStateException("Null setpoint!");

        // use the goal nearest to the measurement.
        goal = new Model100(
                m_modulus.applyAsDouble(goal.x() - measurement.x()) + measurement.x(),
                goal.v());

        // adjust the setpoint based on the measurement.
        m_setpoint = new Model100(
                m_modulus.applyAsDouble(m_setpoint.x() - measurement.x()) + measurement.x(),
                m_setpoint.v());

        // Feedback compares the current measurement with the current profile.
        double u_FB = m_feedback.calculate(measurement, m_setpoint);

        // Profile result is for the next time step.
        ResultWithETA result = m_profile.calculateWithETA(kDt, m_setpoint, goal);

        Control100 u_FF = result.state();
        m_setpoint = u_FF.model();

        return new Result(u_FF, u_FB);
    }

    public Model100 getSetpoint() {
        return m_setpoint;
    }

    /**
     * The feedback controller error is within its tolerance, i.e. we are following
     * the profile well.
     */
    public boolean atSetpoint() {
        boolean atSetpoint = m_feedback.atSetpoint();
        // Util.printf("profiled controller at setpoint %b\n", atSetpoint);
        return atSetpoint;
    }

    /**
     * The profile has reached the goal and the feedback error is within tolerance,
     * i.e. our path is complete.
     * 
     * This doesn't use current measurements, it uses whatever inputs we saw in
     * calculate() most recently.
     */
    public boolean atGoal(Model100 goal) {
        Model100 setpoint = getSetpoint();
        // Util.printf("setpoint %s\n", setpoint);
        return atSetpoint()
                && MathUtil.isNear(
                        goal.x(),
                        setpoint.x(),
                        kPositionTolerance)
                && MathUtil.isNear(
                        goal.v(),
                        setpoint.v(),
                        kVelocityTolerance);
    }
}
