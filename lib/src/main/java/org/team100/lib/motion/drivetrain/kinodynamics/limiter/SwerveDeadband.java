package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

/**
 * Ignores very small inputs.
 * 
 * There might be a smarter way to do this, e.g. look at the velocity over time,
 * to differentiate "noise around zero" from "small trend".
 * 
 * It also might be smarter to deadband noise at the source instead of the
 * output, e.g. if the camera is producing low-level noise, it could be
 * deadbanded in the pose estimator.
 * 
 * For now, this is the simplest thing I could think of.
 */
public class SwerveDeadband implements Glassy {
    /** 1 cm/s */
    private final double m_translationLimit = 0.01;
    /** 0.01 rad/s */
    private final double m_omegaLimit = 0.001;
    private final DoubleLogger m_log_scale;

    public SwerveDeadband(LoggerFactory parent) {
        LoggerFactory child = parent.child(this);
        m_log_scale = child.doubleLogger(Level.TRACE, "scale");
    }

    public FieldRelativeVelocity apply(FieldRelativeVelocity target) {
        if (Math.abs(target.x()) > m_translationLimit
                || Math.abs(target.y()) > m_translationLimit
                || Math.abs(target.theta()) > m_omegaLimit) {
            m_log_scale.log(() -> 1.0);
            return target;
        }
        m_log_scale.log(() -> 0.0);
        return FieldRelativeVelocity.kZero;
    }
}
