package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.controller.simple.IncrementalProfiledController;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Model100Logger;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.MathUtil;

/**
 * Position control using duty cycle feature of linear mechanism
 */
public class OnboardLinearDutyCyclePositionServo implements LinearPositionServo {
    private final LinearMechanism m_mechanism;
    private final ProfiledController m_controller;
    private final double m_kV;
    // LOGGERS
    private final Model100Logger m_log_goal;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;
    private final Control100Logger m_log_setpoint;
    private final DoubleLogger m_log_u_FB;
    private final DoubleLogger m_log_u_FF;
    private final DoubleLogger m_log_u_TOTAL;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_velocity_error;
    

    public OnboardLinearDutyCyclePositionServo(
            LoggerFactory parent,
            LinearMechanism mechanism,
            ProfiledController controller,
            double kV) {
        LoggerFactory child = parent.child(this);
        m_mechanism = mechanism;
        m_controller = controller;
        m_kV = kV;

        m_log_goal = child.model100Logger(Level.TRACE, "goal (m)");
        m_log_position = child.doubleLogger(Level.TRACE, "position (m)");
        m_log_velocity = child.doubleLogger(Level.TRACE, "velocity (m_s)");
        m_log_setpoint = child.control100Logger(Level.TRACE, "setpoint (m)");
        m_log_u_FB = child.doubleLogger(Level.TRACE, "u_FB (duty cycle)");
        m_log_u_FF = child.doubleLogger(Level.TRACE, "u_FF (duty cycle)");
        m_log_u_TOTAL = child.doubleLogger(Level.TRACE, "u_TOTAL (duty cycle)");
        m_log_error = child.doubleLogger(Level.TRACE, "Controller Position Error (m)");
        m_log_velocity_error = child.doubleLogger(Level.TRACE, "Controller Velocity Error (m_s)");
    }

    @Override
    public void reset() {
        OptionalDouble position = getPosition();
        if (position.isEmpty())
            return;
        // using the current velocity sometimes includes a whole lot of noise, and then
        // the profile tries to follow that noise. so instead, use zero.
        // OptionalDouble velocity = getVelocity();
        // if (velocity.isEmpty())
        // return;
        m_controller.init(new Model100(position.getAsDouble(), 0));
    }

    @Override
    public void setPositionWithVelocity(
            double goalM,
            double goalVelocityM_S,
            double feedForwardTorqueNm) {
        final OptionalDouble position = getPosition();
        final OptionalDouble velocity = getVelocity();
        if (position.isEmpty() || velocity.isEmpty())
            return;
        final Model100 measurement = new Model100(position.getAsDouble(), velocity.getAsDouble());
        final Model100 goal = new Model100(goalM, goalVelocityM_S);
        final ProfiledController.Result result = m_controller.calculate(measurement, goal);
        final Control100 setpoint = result.feedforward();
        final double u_FF = m_kV * setpoint.v();
        final double u_FB = result.feedback();
        final double u_TOTAL = MathUtil.clamp(u_FF + u_FB, -1.0, 1.0);
        m_mechanism.setDutyCycle(u_TOTAL);

        m_log_goal.log(() -> goal);
        m_log_setpoint.log(() -> setpoint);
        m_log_u_FB.log(() -> u_FB);
        m_log_u_FF.log(() -> u_FF);
        m_log_u_TOTAL.log(() -> u_TOTAL);
        m_log_error.log(() -> setpoint.x() - position.getAsDouble());
        m_log_velocity_error.log(() -> setpoint.v() - velocity.getAsDouble());
    }

    @Override
    public void setPosition(double goalM, double feedForwardTorqueNm) {
        setPositionWithVelocity(goalM, 0, feedForwardTorqueNm);
    }

    @Override
    public OptionalDouble getPosition() {
        return m_mechanism.getPositionM();
    }

    @Override
    public OptionalDouble getVelocity() {
        return m_mechanism.getVelocityM_S();
    }

    @Override
    public boolean profileDone() {
        return m_controller.profileDone();
    }

    @Override
    public void stop() {
        m_mechanism.stop();
    }

    @Override
    public void close() {
        //
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
        m_log_position.log(() -> getPosition().orElse(Double.NaN));
        m_log_velocity.log(() -> getVelocity().orElse(Double.NaN));
    }

}
