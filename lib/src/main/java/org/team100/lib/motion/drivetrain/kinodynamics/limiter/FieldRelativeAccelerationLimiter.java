package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeAccelerationLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Util;

/**
 * Limits cartesian (a) and rotational (α) acceleration.
 * 
 * The limit is applied to the entire holonomic acceleration together, not to
 * the individual components, so if you exceed the rotational limit, both
 * rotation *and* translational speed will be reduced.
 */
public class FieldRelativeAccelerationLimiter implements Glassy {
    private static final boolean DEBUG = false;
    
    private final DoubleLogger m_log_scale;
    private final FieldRelativeAccelerationLogger m_log_accel;
    private final SwerveKinodynamics m_limits;
    private final double m_cartesianScale;
    private final double m_alphaScale;

    /**
     * @param parent
     * @param limits         Absolute maxima.
     * @param cartesianScale Fraction of cartesian maximum to allow.
     * @param alphaScale     Fraction of rotational maximum to allow.
     */
    public FieldRelativeAccelerationLimiter(
            LoggerFactory parent,
            SwerveKinodynamics limits,
            double cartesianScale,
            double alphaScale) {
        LoggerFactory child = parent.child(this);
        m_log_scale = child.doubleLogger(Level.TRACE, "scale");
        m_log_accel = child.fieldRelativeAccelerationLogger(Level.TRACE, "accel");
        m_limits = limits;
        m_cartesianScale = cartesianScale;
        m_alphaScale = alphaScale;
    }

    public FieldRelativeVelocity apply(
            FieldRelativeVelocity prev,
            FieldRelativeVelocity target) {
        // Acceleration required to achieve the target.
        FieldRelativeAcceleration accel = target.accel(
                prev,
                TimedRobot100.LOOP_PERIOD_S);
        m_log_accel.log(() -> accel);
        double cartesianScale = cartesianScale(prev, target, accel);
        double alphaScale = alphaScale(accel);
        double scale = Math.min(cartesianScale, alphaScale);
        m_log_scale.log(() -> scale);
        FieldRelativeVelocity result = prev.plus(accel.times(scale).integrate(TimedRobot100.LOOP_PERIOD_S));
        if (DEBUG)
            Util.printf(
                    "FieldRelativeAccelerationLimiter prev %s target %s accel %s cartesian scale %5.2f alpha scale %5.2f total scale %5.2f result %s\n",
                    prev, target, accel, cartesianScale, alphaScale, scale, result);
        return result;
    }

    double cartesianScale(
            FieldRelativeVelocity prev,
            FieldRelativeVelocity target,
            FieldRelativeAcceleration accel) {
        double a = accel.norm();
        if (Math.abs(a) < 1e-6) {
            // Avoid divide-by-zero.
            return 1.0;
        }
        double accelLimit = SwerveUtil.getAccelLimit(m_limits, 1, m_cartesianScale, prev, target);
        // At full speed, both a and accelLimit are around zero.
        if (a < accelLimit) {
            a = accelLimit;
        }
        return Math.min(1, accelLimit / a);
    }

    private double alphaScale(FieldRelativeAcceleration accel) {
        double a = accel.theta();
        if (Math.abs(a) < 1e-6) {
            // Avoid divide-by-zero.
            return 1.0;
        }
        double accelLimit = m_alphaScale * m_limits.getMaxAngleAccelRad_S2();
        if (a < accelLimit) {
            a = accelLimit;
        }
        return Math.min(1, accelLimit / a);
    }

}
