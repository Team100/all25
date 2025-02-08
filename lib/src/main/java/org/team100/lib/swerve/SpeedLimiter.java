package org.team100.lib.swerve;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;

/**
 * Enforces maximum wheel speeds.
 */
public class SpeedLimiter implements Glassy {

    private final SwerveKinodynamics m_limits;
    // LOGGERS
    private final DoubleLogger m_log_max_step;
    private final DoubleLogger m_log_s;

    public SpeedLimiter(LoggerFactory parent, SwerveKinodynamics limits) {
        LoggerFactory child = parent.child(this);
        m_limits = limits;
        m_log_max_step = child.doubleLogger(Level.TRACE, "max_vel_step");
        m_log_s = child.doubleLogger(Level.TRACE, "s");
    }

    public double enforceSpeedLimit(SwerveModuleStates states) {
        double limit = m_limits.getMaxDriveVelocityM_S();
        double desired = states.maxSpeed();
        if (desired <= limit)
            return 1.0;
        double scale = limit / desired;
        return scale;
    }

}
