package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * Profiled or direct position control using the feedback controller in the
 * motor controller hardware.
 */
public class OutboardLinearPositionServo implements LinearPositionServo {
    private static final double kPositionTolerance = 0.01;
    private static final double kVelocityTolerance = 0.01;
    private final LinearMechanism m_mechanism;
    private final ProfileReference1d m_ref;

    private final DoubleLogger m_log_goal;
    private final DoubleLogger m_log_ff_torque;
    private final Control100Logger m_log_control;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;

    /** Null if there's no current profile. */
    private Model100 m_goal;
    // TODO: should this be both? which one should we retain?
    private Control100 m_nextSetpoint;

    // for calculating acceleration
    private double previousSetpoint = 0;

    public OutboardLinearPositionServo(
            LoggerFactory parent,
            LinearMechanism mechanism,
            ProfileReference1d ref) {
        LoggerFactory child = parent.child(this);
        m_mechanism = mechanism;
        m_ref = ref;
        m_log_goal = child.doubleLogger(Level.COMP, "goal (m)");
        m_log_ff_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_control = child.control100Logger(Level.COMP, "control (m)");
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
        Control100 measurement = new Control100(position.getAsDouble(), 0);
        m_nextSetpoint = measurement;
        // reference is initalized with measurement only here.
        m_ref.setGoal(measurement.model());
        m_ref.init(measurement.model());
    }

    /** Resets the profile if necessary */
    @Override
    public void setPositionProfiled(double goalM, double feedForwardTorqueNm) {
        m_log_goal.log(() -> goalM);
        Model100 goal = new Model100(goalM, 0);

        if (!goal.near(m_goal, kPositionTolerance, kVelocityTolerance)) {
            m_goal = goal;
            m_ref.setGoal(goal);
            if (m_nextSetpoint == null) {
                // erased by dutycycle control
                OptionalDouble position = getPosition();
                if (position.isEmpty())
                    return;
                m_nextSetpoint = new Control100(position.getAsDouble(), 0);
            }
            // initialize with the setpoint, not the measurement, to avoid noise.
            m_ref.init(m_nextSetpoint.model());
        }
        actuate(m_ref.get(), feedForwardTorqueNm);
    }

    /** Invalidates the current profile */
    @Override
    public void setPositionDirect(Setpoints1d setpoints, double feedForwardTorqueNm) {
        m_goal = null;
        actuate(setpoints, feedForwardTorqueNm);
    }

    /**
     * Pass the setpoint directly to the mechanism's position controller.
     * For outboard control we only use the "next" setpoint.
     */
    private void actuate(Setpoints1d setpoints, double feedForwardTorqueNm) {
        // setpoint must be updated so the profile can see it
        m_nextSetpoint = setpoints.next();
        m_mechanism.setPosition(m_nextSetpoint.x(), m_nextSetpoint.v(), m_nextSetpoint.a(), feedForwardTorqueNm);
        m_log_control.log(() -> m_nextSetpoint);
        m_log_ff_torque.log(() -> feedForwardTorqueNm);
    }

    @Override
    public OptionalDouble getPosition() {
        return m_mechanism.getPositionM();
    }

    public void setDutyCycle(double value) {
        m_goal = null;
        m_nextSetpoint = null;
        m_mechanism.setDutyCycle(value);
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
