package org.team100.lib.swerve;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Enforces a fixed limit on delta v.
 */
public class CapsizeAccelerationLimiter implements Glassy {
    private static final boolean DEBUG = false;
    private final SwerveKinodynamics m_limits;
    private final DoubleLogger m_log_s;

    public CapsizeAccelerationLimiter(LoggerFactory parent, SwerveKinodynamics limits) {
        LoggerFactory child = parent.child(this);
        m_limits = limits;
        m_log_s = child.doubleLogger(Level.TRACE, "s");
    }

    /**
     * 
     * @param dx difference between desired and actual speed, m/s
     * @param dy
     * @return
     */
    public double enforceCentripetalLimit(ChassisSpeeds prev, ChassisSpeeds target) {
        final double dx = target.vxMetersPerSecond - prev.vxMetersPerSecond;
        final double dy = target.vyMetersPerSecond - prev.vyMetersPerSecond;

        double min_s = 1.0;
        double dv = Math.hypot(dx, dy);
        double a = dv / TimedRobot100.LOOP_PERIOD_S;
        if (Math.abs(dv) > 1e-6) {
            min_s = Math.min(1, m_limits.getMaxCapsizeAccelM_S2() / a);
        }
        double s = min_s;
        m_log_s.log(() -> s);
        if (DEBUG)
            Util.printf("a %f limit %f\n", a, m_limits.getMaxCapsizeAccelM_S2());
        return s;
    }

}
