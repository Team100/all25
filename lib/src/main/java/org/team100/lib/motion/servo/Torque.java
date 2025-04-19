package org.team100.lib.motion.servo;

import java.util.OptionalDouble;
import java.util.function.DoubleUnaryOperator;

import org.team100.lib.util.Util;

/** Represents extra torque for gravity and/or spring compensation. */
public class Torque {
    final DoubleUnaryOperator m_torque;

    public Torque(DoubleUnaryOperator torque) {
        m_torque = torque;
    }

    /**
     * I know it's weird to use an optional argument here but it keeps the default
     * in one place.
     */
    public double torque(OptionalDouble optPos) {
        if (optPos.isEmpty()) {
            Util.warn("GravityServo: Broken sensor!");
            // zero is an acceptable torque value in this case.
            return 0;
        }
        double mechanismPositionRad = optPos.getAsDouble();
        return m_torque.applyAsDouble(mechanismPositionRad);
    }

}
