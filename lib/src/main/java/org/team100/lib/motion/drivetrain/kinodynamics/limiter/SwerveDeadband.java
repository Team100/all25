package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

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
public class SwerveDeadband {
    /** 1 cm/s */
    private final double m_translationLimit = 0.01;
    /** 0.01 rad/s */
    private final double m_omegaLimit = 0.001;

    public FieldRelativeVelocity apply(FieldRelativeVelocity target) {
        if (Math.abs(target.x()) > m_translationLimit)
            return target;
        if (Math.abs(target.y()) > m_translationLimit)
            return target;
        if (Math.abs(target.theta()) > m_omegaLimit)
            return target;

        return FieldRelativeVelocity.kZero;
    }
}
