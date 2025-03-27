package org.team100.lib.controller.simple;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.profile.timed.TimedProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Takt;

import edu.wpi.first.math.MathUtil;

/**
 * Feedback and feedforward control.
 * 
 * TODO: dedupe the two profiled controllers.
 */
public class TimedProfiledController implements ProfiledController {
    private final TimedProfile m_profile;
    private final Feedback100 m_feedback;
    private final DoubleUnaryOperator m_modulus;
    private final double m_positionTolerance;
    private final double m_velocityTolerance;

    private double m_startTimeS;
    private Model100 m_setpoint;
    private Model100 m_goal;

    public TimedProfiledController(
            TimedProfile profile,
            Feedback100 feedback,
            DoubleUnaryOperator modulus,
            double positionTolerance,
            double velocityTolerance) {
        m_profile = profile;
        m_feedback = feedback;
        m_modulus = modulus;
        m_positionTolerance = positionTolerance;
        m_velocityTolerance = velocityTolerance;
    }

    @Override
    public void init(Model100 measurement) {
        m_startTimeS = Takt.get();
        m_setpoint = measurement;
        m_feedback.reset();
    }

    @Override
    public Result calculate(Model100 measurement, Model100 goal) {
        if (m_goal == null || m_goal != goal) {
            m_profile.init(measurement, goal);
            m_goal = goal;
        }
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
        double t = progress();
        Control100 u_FF = m_profile.sample(t);

        m_setpoint = u_FF.model();

        return new Result(u_FF, u_FB);
    }

    @Override
    public Model100 getSetpoint() {
        return m_setpoint;
    }

    @Override
    public boolean atSetpoint() {
        boolean atSetpoint = m_feedback.atSetpoint();
        return atSetpoint;
    }

    @Override
    public boolean atGoal(Model100 goal) {
        Model100 setpoint = getSetpoint();
        // Util.printf("setpoint %s\n", setpoint);
        return atSetpoint()
                && MathUtil.isNear(
                        goal.x(),
                        setpoint.x(),
                        m_positionTolerance)
                && MathUtil.isNear(
                        goal.v(),
                        setpoint.v(),
                        m_velocityTolerance);
    }

    @Override
    public void close() {
        //
    }

    private double progress() {
        return Takt.get() - m_startTimeS;
    }

}
