package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeAccelerationLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Util;

/**
 * Limits acceleration to avoid tipping over.
 */
public class FieldRelativeCapsizeLimiter implements Glassy {
    private static final boolean DEBUG = false;

    private final DoubleLogger m_log_scale;
    private final FieldRelativeAccelerationLogger m_log_accel;
    private final FieldRelativeVelocityLogger m_log_prev;
    private final FieldRelativeVelocityLogger m_log_target;

    private final SwerveKinodynamics limits;

    public FieldRelativeCapsizeLimiter(
            LoggerFactory parent,
            SwerveKinodynamics m_limits) {
        LoggerFactory child = parent.child(this);
        m_log_scale = child.doubleLogger(Level.TRACE, "scale");
        m_log_accel = child.fieldRelativeAccelerationLogger(Level.TRACE, "accel");
        m_log_prev = child.fieldRelativeVelocityLogger(Level.TRACE, "prev");
        m_log_target = child.fieldRelativeVelocityLogger(Level.TRACE, "target");
        limits = m_limits;
    }

    public FieldRelativeVelocity apply(
            FieldRelativeVelocity prev,
            FieldRelativeVelocity target) {
        m_log_prev.log(() -> prev);
        m_log_target.log(() -> target);
        // Acceleration required to achieve the target.
        FieldRelativeAcceleration accel = target.accel(
                prev,
                TimedRobot100.LOOP_PERIOD_S);
        m_log_accel.log(() -> accel);
        double a = accel.norm();
        if (a < 1e-6) {
            // Zero acceleration.
            a = 0;
        }
        double scale = scale(a);
        m_log_scale.log(() -> scale);
        FieldRelativeVelocity result = prev.plus(accel.times(scale).integrate(TimedRobot100.LOOP_PERIOD_S));
        if (DEBUG)
            Util.printf("FieldRelativeCapsizeLimiter prev %s target %s accel %s scale %5.2f result %s\n",
                    prev, target, accel, scale, result);
        return result;
    }

    double scale(double a) {
        return Math.min(1, limits.getMaxCapsizeAccelM_S2() / a);
    }
}
