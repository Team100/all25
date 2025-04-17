package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Model100Logger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

/**
 * Because the 2025 angular encoder classes do not wind up, this is a version of
 * the position servo that understands that; it's almost a copy of
 * OnboardPositionServo.
 */
public class OnboardAngularPositionServo implements AngularPositionServo {
    private static final boolean DEBUG = false;
    private static final double kPositionTolerance = 0.02;
    private static final double kVelocityTolerance = 0.02;

    private final RotaryMechanism m_mechanism;
    private final ProfileReference1d m_ref;
    private final Feedback100 m_feedback;

    private final DoubleLogger m_log_goal;
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

    private Model100 m_goal;
    // most-recent "next" setpoint
    Control100 m_setpoint;

    public OnboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            ProfileReference1d ref,
            Feedback100 feedback) {
        LoggerFactory child = parent.child(this);
        m_mechanism = mech;
        m_ref = ref;
        m_feedback = feedback;

        m_log_goal = child.doubleLogger(Level.COMP, "goal (rad)");
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
        // TODO: do i need this somewhere else?
        // m_controller.init(new Model100(position.getAsDouble(), 0));
        // System.out.println("IM BEING RESET TO" + position.getAsDouble() +
        // "***********************************************************");
        Control100 measurement = new Control100(position.getAsDouble(), 0);
        m_setpoint = measurement;
        // measurement is used for initialization only here.
        m_ref.setGoal(measurement.model());
        m_ref.init(measurement.model());
        m_feedback.reset();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        m_goal = null;
        m_setpoint = null;
        m_mechanism.setDutyCycle(dutyCycle);
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_mechanism.setTorqueLimit(torqueNm);
    }

    @Override
    public void setPositionProfiled(double goalRad, double feedForwardTorqueNm) {
        m_log_goal.log(() -> goalRad);

        Model100 goal = new Model100(goalRad, 0);
        if (!goal.near(m_goal, kPositionTolerance, kVelocityTolerance)) {
            m_goal = goal;
            m_ref.setGoal(goal);
            if (m_setpoint == null) {
                final OptionalDouble position = m_mechanism.getPositionRad();
                if (position.isEmpty()) {
                    Util.warn("Broken sensor!");
                    return;
                }
                double measurement = position.getAsDouble();
                // avoid velocity noise here
                m_setpoint = new Control100(measurement, 0);
            } else {
                m_setpoint = new Control100(mod(m_setpoint.x()), m_setpoint.v());
            }
            m_ref.init(m_setpoint.model());

        }
        actuate(m_ref.get(), feedForwardTorqueNm);
    }

    /**
     * set mechanism velocity use the current setpoint for feedback, and
     * the next setpoint velocity as feedforward.
     */
    @Override
    public void setPositionDirect(Setpoints1d setpoint, double feedForwardTorqueNm) {
        m_goal = null;
        actuate(setpoint, feedForwardTorqueNm);
    }

    private void actuate(Setpoints1d setpoints, double feedForwardTorqueNm) {
        m_setpoint = setpoints.next();

        final OptionalDouble position = m_mechanism.getPositionRad();
        final OptionalDouble velocity = m_mechanism.getVelocityRad_S();
        if (position.isEmpty() || velocity.isEmpty()) {
            Util.warn("Broken sensor!");
            return;
        }
        final Model100 measurement = new Model100(position.getAsDouble(), velocity.getAsDouble());

        final double u_FB = m_feedback.calculate(
                measurement,
                new Model100(mod(setpoints.current().x()), setpoints.current().v()));
        final double u_FF = m_setpoint.v();
        final double u_TOTAL = u_FB + u_FF;

        m_mechanism.setVelocity(u_TOTAL, m_setpoint.a(), feedForwardTorqueNm);

        m_log_feedforward_torque.log(() -> feedForwardTorqueNm);
        m_log_measurement.log(() -> measurement);
        m_log_control.log(() -> m_setpoint);
        m_log_u_FB.log(() -> u_FB);
        m_log_u_FF.log(() -> u_FF);
        m_log_u_TOTAL.log(() -> u_TOTAL);
        m_log_error.log(() -> setpoints.current().x() - position.getAsDouble());
        m_log_velocity_error.log(() -> setpoints.current().v() - velocity.getAsDouble());
    }

    /** Return an angle near the measurement */
    private double mod(double x) {
        OptionalDouble posOpt = m_mechanism.getPositionRad();
        if (posOpt.isEmpty())
            return x;
        double measurement = posOpt.getAsDouble();
        return MathUtil.angleModulus(x - measurement) + measurement;
    }

    /**
     * @return the absolute 1:1 position of the mechanism in [-pi, pi]
     */
    @Override
    public OptionalDouble getPosition() {
        return m_mechanism.getPositionRad();
    }

    @Override
    public boolean atSetpoint() {
        boolean atSetpoint = m_feedback.atSetpoint();
        m_log_at_setpoint.log(() -> atSetpoint);
        return atSetpoint;
    }

    @Override
    public boolean profileDone() {
        if (m_goal == null) {
            // if there's no profile, it's always done.
            return true;
        }
        return m_ref.profileDone();
    }

    @Override
    public boolean atGoal() {
        return atSetpoint() && profileDone();
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
