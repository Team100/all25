package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.MathUtil;

/**
 * Position control using duty cycle feature of linear mechanism
 */
public class OnboardLinearDutyCyclePositionServo implements LinearPositionServo {
    private static final double kPositionTolerance = 0.01;
    private static final double kVelocityTolerance = 0.01;
    private final LinearMechanism m_mechanism;
    private final ProfileReference1d m_ref;
    private final Feedback100 m_feedback;
    private final double m_kV;
    private final DoubleLogger m_log_goal;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;
    private final Control100Logger m_log_setpoint;
    private final DoubleLogger m_log_u_FB;
    private final DoubleLogger m_log_u_FF;
    private final DoubleLogger m_log_u_TOTAL;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_velocity_error;

    /** Null if there's no current profile. */
    private Model100 m_goal;
    // TODO: should this be both? which one should we retain?
    private Control100 m_setpoint;

    public OnboardLinearDutyCyclePositionServo(
            LoggerFactory parent,
            LinearMechanism mechanism,
            ProfileReference1d ref,
            Feedback100 feedback,
            double kV) {
        LoggerFactory child = parent.child(this);
        m_mechanism = mechanism;
        m_ref = ref;
        m_feedback = feedback;
        m_kV = kV;

        m_log_goal = child.doubleLogger(Level.TRACE, "goal (m)");
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
        Control100 measurement = new Control100(position.getAsDouble(), 0);
        m_setpoint = measurement;
        m_ref.setGoal(measurement.model());
        // reference is initalized with measurement only here.
        m_ref.init(measurement.model());
        // m_controller.init(m_setpoint.model());
        m_feedback.reset();
    }

    /**
     * Resets the profile if necessary.
     * 
     * @param goalM
     * @param feedForwardTorqueNm ignored
     */
    @Override
    public void setPositionProfiled(double goalM, double feedForwardTorqueNm) {
        m_log_goal.log(() -> goalM);
        final Model100 goal = new Model100(goalM, 0);

        if (!goal.near(m_goal, kPositionTolerance, kVelocityTolerance)) {
            m_goal = goal;
            m_ref.setGoal(goal);
            // initialize with the setpoint, not the measurement, to avoid noise.
            m_ref.init(m_setpoint.model());
        }
        actuate(m_ref.get(), feedForwardTorqueNm);
    }

    /**
     * Invalidates the current profile
     * 
     * @param setpoints
     * @param feedForwardTorqueNm ignored
     */
    @Override
    public void setPositionDirect(Setpoints1d setpoints, double feedForwardTorqueNm) {
        m_goal = null;
        actuate(setpoints, feedForwardTorqueNm);
    }

    /**
     * Compute feedback using the current setpoint, feedforward using the next
     * setpoint, and actuate using duty cycle.
     * Ignores torque
     */
    private void actuate(Setpoints1d setpoints, double feedForwardTorqueNm) {
        // setpoint must be updated so the profile can see it
        m_setpoint = setpoints.next();

        final OptionalDouble position = getPosition();
        final OptionalDouble velocity = getVelocity();
        if (position.isEmpty() || velocity.isEmpty())
            return;
        final Model100 measurement = new Model100(position.getAsDouble(), velocity.getAsDouble());
        final double u_FF = m_kV * m_setpoint.v();
        final double u_FB = m_feedback.calculate(measurement, setpoints.current().model());
        final double u_TOTAL = MathUtil.clamp(u_FF + u_FB, -1.0, 1.0);

        m_mechanism.setDutyCycle(u_TOTAL);

        m_log_setpoint.log(() -> m_setpoint);

        m_log_u_FB.log(() -> u_FB);
        m_log_u_FF.log(() -> u_FF);
        m_log_u_TOTAL.log(() -> u_TOTAL);
        m_log_error.log(() -> setpoints.current().x() - position.getAsDouble());
        m_log_velocity_error.log(() -> setpoints.current().v() - velocity.getAsDouble());
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
        if (m_goal == null) {
            // if there's no profile, it's always done.
            return true;
        }
        return m_ref.profileDone();
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

    @Override
    public boolean atGoal(double xTolerance, double vTolerance) {
        return Math.abs(m_goal.x() - getPosition().getAsDouble()) < xTolerance &&
            Math.abs(m_goal.v() - getVelocity().getAsDouble()) < vTolerance;
    }
}
