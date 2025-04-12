package org.team100.lib.motion.servo;

import java.util.OptionalDouble;
import java.util.function.DoubleUnaryOperator;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.state.Control100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Wraps an angular position servo, supplying it with the correct feed forward
 * torque for gravity compensation.
 */
public class OutboardGravityServo implements GravityServoInterface {
    public static class Gravity implements DoubleUnaryOperator, Glassy {
        /** Max gravity torque, newton-meters */
        private final double m_gravityNm;
        /** Offset from horizontal */
        private final double m_offsetRad;
        private final DoubleLogger m_log_gravityTorque;
        private final DoubleLogger m_log_correctedPosition;

        public Gravity(LoggerFactory parent, double gravityNm, double offsetRad) {
            m_gravityNm = gravityNm;
            m_offsetRad = offsetRad;
            LoggerFactory child = parent.child(this);
            m_log_gravityTorque = child.doubleLogger(Level.TRACE, "gravity torque (Nm)");
            m_log_correctedPosition = child.doubleLogger(Level.TRACE, "corrected position (rad)");
        }

        @Override
        public double applyAsDouble(double mechanismPositionRad) {
            double correctedPosition = mechanismPositionRad + m_offsetRad;
            double gravityTorqueNm = m_gravityNm * -Math.sin(correctedPosition);
            m_log_correctedPosition.log(() -> correctedPosition);
            m_log_gravityTorque.log(() -> gravityTorqueNm);
            return gravityTorqueNm;
        }
    }

    public static class Spring implements DoubleUnaryOperator, Glassy {
        // elevator height, elevator cg
        private InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
        private final DoubleLogger m_log_springTorque;

        public Spring(LoggerFactory parent) {
            LoggerFactory child = parent.child(this);
            m_log_springTorque = child.doubleLogger(Level.TRACE, "spring torque (Nm)");

            table.put(-0.113302, -10.0);
            table.put(0.34, -6.2);
            table.put(0.677192, -5.0);
            table.put(0.784419, -4.0);
            table.put(0.921020, -3.0);
            table.put(1.734179, -1.0);
        }

        @Override
        public double applyAsDouble(double mechanismPositionRad) {
            double springTorqueNm = table.get(mechanismPositionRad);
            m_log_springTorque.log(() -> springTorqueNm);
            return springTorqueNm;
        }

    }

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
        double mechanismPositionRad = optPos.getAsDouble();
        m_servo.setPositionWithVelocity(
                goal.x(), goal.v(), m_torque.applyAsDouble(mechanismPositionRad));
    }

    @Override
    public void stop() {
        m_servo.stop();
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
    public boolean profileDone() {
        return m_servo.profileDone();
    }

    @Override
    public void setStaticTorque(double value) {
        OptionalDouble optPos = getPositionRad();
        if (optPos.isEmpty()) {
            Util.warn("GravityServo: Broken sensor!");
            return;
        }
        double mechanismPositionRad = optPos.getAsDouble();
        mechanismPositionRad = (mechanismPositionRad * 1.16666666667);
        final double gravityTorqueNm = m_torque.applyAsDouble(mechanismPositionRad);
        m_servo.setStaticTorque(gravityTorqueNm);
    }
}
