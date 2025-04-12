package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.framework.TimedRobot100;
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
 * Position control using the feedback controller in the motor controller
 * hardware
 * 
 * TODO: integrate ProfiledController here.
 */
public class OutboardLinearPositionServo implements LinearPositionServo {
    private static final double kPositionTolerance = 0.01;
    private static final double kVelocityTolerance = 0.01;
    private final LinearMechanism m_mechanism;
    private final ProfiledController m_controller;

    private Model100 m_goal;
    private Control100 m_setpoint = new Control100(0, 0);

    private final Model100Logger m_log_goal;
    private final DoubleLogger m_log_ff_torque;
    private final Control100Logger m_log_setpoint;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;

    // for calculating acceleration
    private double previousSetpoint = 0;

    public OutboardLinearPositionServo(
            LoggerFactory parent,
            LinearMechanism mechanism,
            ProfiledController controller) {
        LoggerFactory child = parent.child(this);
        m_mechanism = mechanism;
        m_controller = controller;
        m_log_goal = child.model100Logger(Level.COMP, "goal (m)");
        m_log_ff_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_setpoint = child.control100Logger(Level.COMP, "setpoint (m)");
        m_log_position = child.doubleLogger(Level.COMP, "position (m)");
        m_log_velocity = child.doubleLogger(Level.COMP, "velocity (m_s)");
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
        m_setpoint = new Control100(position.getAsDouble(), 0);
        m_controller.init(new Model100(position.getAsDouble(), 0));
    }

    @Override
    public void setPosition(double goalM, double feedForwardTorqueNm) {

        final OptionalDouble position = getPosition();
        final OptionalDouble velocity = getVelocity();
        if (position.isEmpty() || velocity.isEmpty())
            return;
        final Model100 measurement = new Model100(position.getAsDouble(), velocity.getAsDouble());

        m_goal = new Model100(goalM, 0);
        final ProfiledController.Result result = m_controller.calculate(measurement, m_goal);
        m_setpoint = result.feedforward();

        m_mechanism.setPosition(m_setpoint.x(), m_setpoint.v(), m_setpoint.a(), feedForwardTorqueNm);

        m_log_goal.log(() -> m_goal);
        m_log_ff_torque.log(() -> feedForwardTorqueNm);
        m_log_setpoint.log(() -> m_setpoint);
    }

    /** Set the mechanism to exactly the goal position, without a profile. */
    public void setPositionDirectly(double goalM, double feedForwardTorqueNm) {
        final OptionalDouble position = getPosition();
        final OptionalDouble velocity = getVelocity();
        if (position.isEmpty() || velocity.isEmpty())
            return;
        final Model100 measurement = new Model100(position.getAsDouble(), velocity.getAsDouble());

        // update the controller even though we don't use the result.
        @SuppressWarnings("unused")
        final ProfiledController.Result result = m_controller.calculate(measurement, m_goal);

        m_goal = new Model100(goalM, 0.0);

        m_setpoint = m_goal.control();

        m_mechanism.setPosition(m_setpoint.x(), m_setpoint.v(), m_setpoint.a(), feedForwardTorqueNm);

        m_log_goal.log(() -> m_goal);
        m_log_ff_torque.log(() -> feedForwardTorqueNm);
        m_log_setpoint.log(() -> m_setpoint);
    }

    @Override
    public OptionalDouble getPosition() {
        return m_mechanism.getPositionM();
    }

    public void setDutyCycle(double value) {
        m_mechanism.setDutyCycle(value);
    }

    @Override
    public OptionalDouble getVelocity() {
        return m_mechanism.getVelocityM_S();
    }

    @Override
    public boolean profileDone() {
        // the only way to tell if an incremental profile is done is to compare the
        // setpoint to the goal.
        return MathUtil.isNear(
                m_goal.x(),
                m_setpoint.x(),
                kPositionTolerance)
                && MathUtil.isNear(
                        m_goal.v(),
                        m_setpoint.v(),
                        kVelocityTolerance);
    }

    @Override
    public void stop() {
        m_mechanism.stop();
    }

    @Override
    public void close() {
        m_mechanism.close();
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
        m_log_position.log(() -> getPosition().orElse(Double.NaN));
        m_log_velocity.log(() -> getVelocity().orElse(Double.NaN));
    }

    /**
     * Acceleration from trailing difference in velocity.
     * 
     * To avoid injecting clock noise into the acceleration signal, this uses
     * a constant dt, TimedRobot100.LOOP_PERIOD_S, so you'd better be calling this
     * at about that rate.
     * 
     * @param setpoint desired velocity
     */
    private double accel(double setpoint) {
        double accel = (setpoint - previousSetpoint) / TimedRobot100.LOOP_PERIOD_S;
        previousSetpoint = setpoint;
        return accel;
    }
}
