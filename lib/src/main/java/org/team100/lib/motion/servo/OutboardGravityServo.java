package org.team100.lib.motion.servo;

import java.util.OptionalDouble;
import java.util.logging.Logger;

import org.team100.lib.state.Control100;
import org.team100.lib.util.Util;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

/**
 * Wraps an angular position servo, supplying it with the correct feed forward
 * torque for gravity compensation.
 */
public class OutboardGravityServo implements GravityServoInterface{
    private final AngularPositionServo m_servo;
    /** Max gravity torque, newton-meters */
    private final double m_gravityNm; // = 5.0;
    /** Offset from horizontal */
    private final double m_offsetRad; // = 0.0;
    private final DoubleLogger m_log_gT;
    private final DoubleLogger m_correctedPosition;

    public OutboardGravityServo(
            LoggerFactory parent,
            AngularPositionServo servo,
            double gravityNm,
            double offsetRad) {
        LoggerFactory child = parent.child(this);
        m_servo = servo;
        m_gravityNm = gravityNm;
        m_offsetRad = offsetRad;
        m_log_gT = child.doubleLogger(Level.TRACE, "gravity torque (Nm)");
        m_correctedPosition = child.doubleLogger(Level.TRACE, "corrected position (rad)");
    }

    @Override
    public void reset() {
        m_servo.reset();
    }

    @Override
    public OptionalDouble getPositionRad() {
        return m_servo.getPosition();
    }

    @Override
    public void setState(Control100 goal) {
        OptionalDouble optPos = getPositionRad();
        if (optPos.isEmpty()) {
            Util.warn("GravityServo: Broken sensor!");
            return;
        }
        double mechanismPositionRad = optPos.getAsDouble() / 1.16666666667 ;
        m_correctedPosition.log(() -> mechanismPositionRad);
        final double gravityTorqueNm = m_gravityNm * -Math.sin(mechanismPositionRad + m_offsetRad); //TODO MAKE THIS COS

        m_log_gT.log(() -> gravityTorqueNm);
        m_servo.setPositionWithVelocity(goal.x(), goal.v(), gravityTorqueNm);
    }

    @Override
    public void stop() {
        m_servo.stop();
    }

    @Override
    public void setEncoderPosition(double positionRad) {
        m_servo.setEncoderPosition(positionRad);
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_servo.setTorqueLimit(torqueNm);
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    @Override
    public boolean atSetpoint() {
        return m_servo.atSetpoint();
    }

    @Override
    public void setStaticTorque(double value) {
        OptionalDouble optPos = getPositionRad();
        if (optPos.isEmpty()) {
            Util.warn("GravityServo: Broken sensor!");
            return;
        }
        double mechanismPositionRad = optPos.getAsDouble();
        mechanismPositionRad = (mechanismPositionRad * 1.16666666667); //CHANGE THIS IF YOU WANT TO CHANGE BACK TO WRIST 1 PLEASE
        final double gravityTorqueNm = m_gravityNm * -Math.sin(mechanismPositionRad + m_offsetRad); //TODO MAKE THIS COS
        m_servo.setStaticTorque(gravityTorqueNm);
    }
}
