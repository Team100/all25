package org.team100.lib.motion.servo;

import java.util.OptionalDouble;
import java.util.function.DoubleUnaryOperator;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.util.Util;

/**
 * Wraps an angular position servo, supplying it with the correct feed forward
 * torque for gravity compensation.
 */
public class OutboardGravityServo implements Glassy {
    private final AngularPositionServo m_servo;
    final DoubleUnaryOperator m_torque;

    public OutboardGravityServo(
            LoggerFactory parent,
            AngularPositionServo servo,
            DoubleUnaryOperator torque) {
        LoggerFactory child = parent.child(this);
        m_servo = servo;
        m_torque = torque;
    }

    public void setPosition(double goalRad) {
        OptionalDouble optPos = getPositionRad();
        if (optPos.isEmpty()) {
            Util.warn("GravityServo: Broken sensor!");
            return;
        }
        double mechanismPositionRad = optPos.getAsDouble();
        m_servo.setPositionWithVelocity(
                goalRad, 0, m_torque.applyAsDouble(mechanismPositionRad));
    }

    /** Zeros controller errors, sets setpoint to current position. */
    public void reset() {
        m_servo.reset();
    }

    public OptionalDouble getPositionRad() {
        return m_servo.getPosition();
    }

    public void stop() {
        m_servo.stop();
    }

    public void setTorqueLimit(double torqueNm) {
        m_servo.setTorqueLimit(torqueNm);
    }

    public void periodic() {
        m_servo.periodic();
    }

    public boolean atSetpoint() {
        return m_servo.atSetpoint();
    }

    public boolean profileDone() {
        return m_servo.profileDone();
    }
}
