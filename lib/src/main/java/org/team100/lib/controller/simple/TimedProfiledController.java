package org.team100.lib.controller.simple;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.Model100Logger;
import org.team100.lib.profile.timed.TimedProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Takt;

/**
 * Feedback and feedforward control.
 * 
 * TODO: dedupe the two profiled controllers.
 */
public class TimedProfiledController implements ProfiledController, Glassy {
    private final TimedProfile m_profile;
    private final Feedback100 m_feedback;
    private final DoubleUnaryOperator m_modulus;
    private final double m_positionTolerance;
    private final double m_velocityTolerance;

    private final Model100Logger m_log_setpoint;
    private final Control100Logger m_log_control;

    private double m_startTimeS;
    private Model100 m_setpoint;
    private Model100 m_goal;

    public TimedProfiledController(
            LoggerFactory logger,
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
        // use the parent logger
        m_log_setpoint = logger.model100Logger(Level.TRACE, "setpoint");
        m_log_control = logger.control100Logger(Level.TRACE, "control");
    }

    @Override
    public void init(Model100 measurement) {
        m_startTimeS = Takt.get();
        m_setpoint = measurement;
        m_log_setpoint.log(() -> m_setpoint);
        m_feedback.reset();
    }

    @Override
    public Result calculate(Model100 measurement, Model100 goal) {
        if (m_setpoint == null)
            throw new IllegalStateException("Null setpoint!");

        if (m_goal == null || !m_goal.near(goal, m_velocityTolerance, m_positionTolerance)) {
            // if the goal has changed noticeably, recalculate.
            // use motionless measurement to avoid injecting velocity noise.
            m_profile.init(new Model100(measurement.x(), 0), goal);
            // use the goal nearest to the measurement.
            m_goal = new Model100(
                    m_modulus.applyAsDouble(goal.x() - measurement.x()) + measurement.x(),
                    goal.v());
            m_startTimeS = Takt.get();
        } else {
            // use the goal nearest to the measurement.
            m_goal = new Model100(
                    m_modulus.applyAsDouble(goal.x() - measurement.x()) + measurement.x(),
                    goal.v());
        }

        // adjust the setpoint based on the measurement.
        m_setpoint = new Model100(
                m_modulus.applyAsDouble(m_setpoint.x() - measurement.x()) + measurement.x(),
                m_setpoint.v());

        // Feedback compares the current measurement with the current profile.
        double u_FB = m_feedback.calculate(measurement, m_setpoint);

        // Profile result is for the next time step, LOOP_PERIOD from now.
        Control100 u_FF = m_profile.sample(progress() + TimedRobot100.LOOP_PERIOD_S);
        m_log_control.log(() -> u_FF);
        m_setpoint = u_FF.model();
        m_log_setpoint.log(() -> m_setpoint);
        return new Result(u_FF, u_FB);
    }

    @Override
    public boolean profileDone() {
        // to tell if a timed profile is done, look at the timer.
        return progress() >= m_profile.duration();
    }

    @Override
    public Model100 getSetpoint() {
        return m_setpoint;
    }

    @Override
    public boolean atSetpoint() {
        return m_feedback.atSetpoint();
    }

    @Override
    public boolean atGoal(Model100 goal) {
        return profileDone() && atSetpoint();
    }

    @Override
    public void close() {
        //
    }

    private double progress() {
        return Takt.get() - m_startTimeS;
    }

}
