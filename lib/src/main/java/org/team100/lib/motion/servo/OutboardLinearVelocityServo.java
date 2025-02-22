package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.util.Util;

public class OutboardLinearVelocityServo implements LinearVelocityServo {
    private static final boolean DEBUG = false;
    private final LinearMechanism m_mechanism;
    // LOGGERS
    private final DoubleLogger m_log_setpoint_v;
    private final DoubleLogger m_log_setpoint_a;
    private final OptionalDoubleLogger m_log_velocity;
    private final OptionalDoubleLogger m_log_position;

    // for calculating acceleration
    private double previousSetpoint = 0;
    private double m_setpoint;

    public OutboardLinearVelocityServo(LoggerFactory parent, LinearMechanism mechanism) {
        LoggerFactory child = parent.child(this);
        m_mechanism = mechanism;
        m_log_setpoint_v = child.doubleLogger(Level.TRACE, "setpoint v (m_s)");
        m_log_setpoint_a = child.doubleLogger(Level.TRACE, "setpoint a (m_s2)");
        m_log_velocity = child.optionalDoubleLogger(Level.TRACE, "velocity (m_s)");
        m_log_position = child.optionalDoubleLogger(Level.TRACE, "position (m)");
    }

    @Override
    public void reset() {
        if (DEBUG)
            Util.warn("make sure resetting encoder position doesn't break anything");
        m_mechanism.resetEncoderPosition();
    }

    @Override
    public void setVelocityM_S(double setpointM_S) {
        setVelocity(setpointM_S, accel(setpointM_S));
    }

    @Override
    public void setVelocity(double setpointM_S, double setpointM_S2) {
        m_setpoint = setpointM_S;
        m_mechanism.setVelocity(setpointM_S, setpointM_S2, 0);
        m_log_setpoint_v.log(() -> setpointM_S);
        m_log_setpoint_a.log(() -> setpointM_S2);
    }

    /**
     * @return Current velocity measurement. Note this can be noisy, maybe filter
     *         it.
     */
    @Override
    public OptionalDouble getVelocity() {
        final OptionalDouble velocityM_S = m_mechanism.getVelocityM_S();
        m_log_velocity.log(() -> velocityM_S);
        return velocityM_S;
    }

    @Override
    public void stop() {
        m_mechanism.stop();
    }

    @Override
    public OptionalDouble getDistance() {
        final OptionalDouble positionM = m_mechanism.getPositionM();
        m_log_position.log(() -> positionM);
        return positionM;
    }

    @Override
    public double getSetpoint() {
        return m_setpoint;
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
    }

    ////////////////////////////////////////////////

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
