package org.team100.lib.motion.servo;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

/**
 * Represents torque required to counteract gravity. This was used in 2025 for
 * the wrist joint.
 */
public class Gravity implements DoubleUnaryOperator, Glassy {
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