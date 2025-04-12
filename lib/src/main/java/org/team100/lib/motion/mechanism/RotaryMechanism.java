package org.team100.lib.motion.mechanism;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.BareMotor;

/**
 * Uses a motor and gears to produce rotational output, e.g. an arm joint.
 * 
 * Motor velocity and accel is higher than mechanism, required torque is lower,
 * using the supplied gear ratio.
 * 
 * The position limits used to be enforced by a proxy, but now they're here: it
 * seems simpler that way.
 */
public class RotaryMechanism implements Glassy {
    private final BareMotor m_motor;
    private final RotaryPositionSensor m_sensor;
    private final double m_gearRatio;
    private final double m_minPositionRad;
    private final double m_maxPositionRad;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_position;

    /**
     * The provided sensor encapsulates the motor sensor and/or the external
     * absolute sensor, if used. See ProxyRotaryPositionSensor and
     * CombinedRotaryPositionSensor.
     */
    public RotaryMechanism(
            LoggerFactory parent,
            BareMotor motor,
            RotaryPositionSensor sensor,
            double gearRatio,
            double minPositionRad,
            double maxPositionRad) {
        LoggerFactory child = parent.child(this);
        m_motor = motor;
        m_sensor = sensor;
        m_gearRatio = gearRatio;
        m_minPositionRad = minPositionRad;
        m_maxPositionRad = maxPositionRad;
        m_log_velocity = child.doubleLogger(Level.TRACE, "velocity (rad_s)");
        m_log_position = child.doubleLogger(Level.TRACE, "position (rad)");
    }

    /** Use for homing. */
    public void setDutyCycleUnlimited(double output) {
        m_motor.setDutyCycle(output);
    }

    /** Should actuate immediately. Enforces position limit using the encoder. */
    public void setDutyCycle(double output) {
        OptionalDouble posOpt = getPositionRad();
        if (posOpt.isEmpty()) {
            m_motor.stop();
            return;
        }
        double posRad = posOpt.getAsDouble();
        if (output < 0 && posRad < m_minPositionRad) {
            m_motor.stop();
            return;
        }
        if (output > 0 && posRad > m_maxPositionRad) {
            m_motor.stop();
            return;
        }
        m_motor.setDutyCycle(output);
    }

    public void setTorqueLimit(double torqueNm) {
        m_motor.setTorqueLimit(torqueNm / m_gearRatio);
    }

    /** Should actuate immediately. Use for homing. */
    public void setVelocityUnlimited(
            double outputRad_S,
            double outputAccelRad_S2,
            double outputTorqueNm) {
        m_motor.setVelocity(
                outputRad_S * m_gearRatio,
                outputAccelRad_S2 * m_gearRatio,
                outputTorqueNm / m_gearRatio);
    }

    /** Should actuate immediately. Enforces position limit using the encoder. */
    public void setVelocity(
            double outputRad_S,
            double outputAccelRad_S2,
            double outputTorqueNm) {
        OptionalDouble posOpt = getPositionRad();
        if (posOpt.isEmpty()) {
            m_motor.stop();
            return;
        }
        double posRad = posOpt.getAsDouble();
        if (outputRad_S < 0 && posRad < m_minPositionRad) {
            m_motor.stop();
            return;
        }
        if (outputRad_S > 0 && posRad > m_maxPositionRad) {
            m_motor.stop();
            return;
        }
        m_motor.setVelocity(
                outputRad_S * m_gearRatio,
                outputAccelRad_S2 * m_gearRatio,
                outputTorqueNm / m_gearRatio);
    }

    /**
     * Should actuate immediately. Enforces position limit using the arguments to
     * this function.
     */
    public void setPosition(
            double outputPositionRad,
            double outputVelocityRad_S,
            double outputAccelRad_S2,
            double outputTorqueNm) {
        if (outputPositionRad < m_minPositionRad) {
            m_motor.stop();
            return;
        }
        if (outputPositionRad > m_maxPositionRad) {
            m_motor.stop();
            return;
        }
        m_motor.setPosition(
                outputPositionRad * m_gearRatio,
                outputVelocityRad_S * m_gearRatio,
                outputAccelRad_S2 * m_gearRatio,
                outputTorqueNm / m_gearRatio);
    }

    /** Value is updated in Robot.robotPeriodic(). */
    public OptionalDouble getVelocityRad_S() {
        return m_sensor.getVelocityRad_S();
    }

    /**
     * @return the absolute 1:1 position of the mechanism in [-pi, pi]
     */
    public OptionalDouble getPositionRad() {
        return m_sensor.getPositionRad();
    }

    public void stop() {
        m_motor.stop();
    }

    public void close() {
        m_motor.close();
    }

    public void periodic() {
        m_motor.periodic();
        m_sensor.periodic();
        m_log_velocity.log(() -> getVelocityRad_S().getAsDouble());
        m_log_position.log(() -> getPositionRad().getAsDouble());
    }

}
