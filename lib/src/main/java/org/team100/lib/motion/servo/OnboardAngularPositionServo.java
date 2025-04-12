package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Model100Logger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

/**
 * Because the 2025 angular encoder classes do not wind up, this is a version of
 * the position servo that understands that; it's almost a copy of
 * OnboardPositionServo.
 */
public class OnboardAngularPositionServo implements AngularPositionServo {
    private static final boolean DEBUG = false;
    private static final double kXTolerance = 0.02;
    private static final double kVTolerance = 0.02;

    private final RotaryMechanism m_mechanism;
    final ProfiledController m_controller;

    private final Model100Logger m_log_goal;
    private final DoubleLogger m_log_feedforward_torque;
    private final Model100Logger m_log_measurement;
    private final Control100Logger m_log_control;
    private final DoubleLogger m_log_u_FB;
    private final DoubleLogger m_log_u_FF;
    private final DoubleLogger m_log_u_TOTAL;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_velocity_error;
    private final DoubleLogger m_encoderValue;
    private final BooleanLogger m_log_at_setpoint;

    private Model100 m_goal = new Model100(0, 0);

    public OnboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            ProfiledController controller) {
        LoggerFactory child = parent.child(this);
        m_mechanism = mech;
        m_controller = controller;

        m_log_goal = child.model100Logger(Level.COMP, "goal (rad)");
        m_log_feedforward_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_measurement = child.model100Logger(Level.COMP, "measurement (rad)");
        m_log_control = child.control100Logger(Level.COMP, "control (rad)");
        m_log_u_FB = child.doubleLogger(Level.TRACE, "u_FB (rad_s)");
        m_log_u_FF = child.doubleLogger(Level.TRACE, "u_FF (rad_s)");
        m_encoderValue = child.doubleLogger(Level.TRACE, "Encoder Value");

        m_log_u_TOTAL = child.doubleLogger(Level.TRACE, "u_TOTAL (rad_s)");
        m_log_error = child.doubleLogger(Level.TRACE, "Controller Position Error (rad)");
        m_log_velocity_error = child.doubleLogger(Level.TRACE, "Controller Velocity Error (rad_s)");
        m_log_at_setpoint = child.booleanLogger(Level.TRACE, "At Setpoint");
    }

    /**
     * It is essential to call this after a period of disuse, to prevent transients.
     * 
     * To prevent oscillation, the previous setpoint is used to compute the profile,
     * but there needs to be an initial setpoint.
     */
    @Override
    public void reset() {
        if (DEBUG) {
            Util.println("OnboardAngularPositionServo reset");
        }
        OptionalDouble position = getPosition();
        if (position.isEmpty())
            return;
        // using the current velocity sometimes includes a whole lot of noise, and then
        // the profile tries to follow that noise. so instead, use zero.
        // OptionalDouble velocity = getVelocity();
        // if (velocity.isEmpty())
        // return;
        m_controller.init(new Model100(position.getAsDouble(), 0));
        // System.out.println("IM BEING RESET TO" + position.getAsDouble() +
        // "***********************************************************");
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_mechanism.setTorqueLimit(torqueNm);
    }

    @Override
    public void setPosition(double goalRad, double feedForwardTorqueNm) {

        final OptionalDouble position = getPosition();
        // note the mechanism uses the motor's internal encoder which may be only
        // approximately attached to the the output, via backlash and slack, so these
        // two measurements might not be entirely consistent.
        // on the other hand, using the position sensor to obtain velocity has its own
        // issues: delay and noise.
        final OptionalDouble velocity = m_mechanism.getVelocityRad_S();

        if (position.isEmpty() || velocity.isEmpty()) {
            Util.warn("Broken sensor!");
            return;
        }
        m_goal = new Model100(goalRad, 0);

        Model100 measurement = new Model100(position.getAsDouble(), velocity.getAsDouble());

        ProfiledController.Result result = m_controller.calculate(measurement, m_goal);
        final Control100 setpointRad = result.feedforward();

        final double u_FF = setpointRad.v();
        // note u_FF is rad/s, so a big number, u_FB should also be a big number.
        final double u_FB = result.feedback();

        final double u_TOTAL = (u_FB) + u_FF;

        m_mechanism.setVelocity(u_TOTAL, setpointRad.a(), feedForwardTorqueNm);

        m_log_goal.log(() -> m_goal);
        m_log_feedforward_torque.log(() -> feedForwardTorqueNm);
        m_log_measurement.log(() -> new Model100(position.getAsDouble(), velocity.getAsDouble()));
        m_log_control.log(() -> setpointRad);
        m_log_u_FB.log(() -> u_FB);
        m_log_u_FF.log(() -> u_FF);
        m_log_u_TOTAL.log(() -> u_TOTAL);
        m_log_error.log(() -> setpointRad.x() - position.getAsDouble());
        m_log_velocity_error.log(() -> setpointRad.v() - velocity.getAsDouble());
    }

    /**
     * @return the absolute 1:1 position of the mechanism in [-pi, pi]
     */
    @Override
    public OptionalDouble getPosition() {
        return m_mechanism.getPositionRad();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        m_mechanism.setDutyCycle(dutyCycle);
    }

    @Override
    public boolean atSetpoint() {
        boolean atSetpoint = m_controller.atSetpoint();
        m_log_at_setpoint.log(() -> atSetpoint);
        return atSetpoint;
    }

    @Override
    public boolean profileDone() {
        return m_controller.profileDone();
    }

    /**
     * position and velocity errors are both under their tolerances.
     */
    @Override
    public boolean atGoal() {
        OptionalDouble position = m_mechanism.getPositionRad();
        OptionalDouble velocity = m_mechanism.getVelocityRad_S();
        if (position.isEmpty() || velocity.isEmpty()) {
            Util.warn("Broken sensor!");
            return false;
        }
        Model100 measurement = new Model100(position.getAsDouble(), velocity.getAsDouble());
        return measurement.near(m_goal, kXTolerance, kVTolerance);
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
        m_encoderValue.log(() -> getPosition().getAsDouble());
    }
}
