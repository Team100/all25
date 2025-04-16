package org.team100.lib.motion.servo;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Represents torque required to counteract a spring. This was used in 2025 for
 * the wrist joint, to try to bias the lash. It's like kinda a straight line,
 * but not exactly.
 * https://docs.google.com/spreadsheets/d/1xRXfkmg8WwagG4IE5CBNJi9Cfs16iEP-pp94ZeJAe6o/edit?gid=0#gid=0
 */
public class Spring implements DoubleUnaryOperator, Glassy {
    // elevator height, elevator cg
    private final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
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