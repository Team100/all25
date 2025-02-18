package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

/**
 * Return feasible velocity, using a simple worst-case model (diagonal course).
 */
public class FieldRelativeVelocityLimiter {
    private final BatterySagSpeedLimit m_limits;

    public FieldRelativeVelocityLimiter(BatterySagSpeedLimit limit) {
        m_limits = limit;
    }

    public FieldRelativeVelocity limit(FieldRelativeVelocity target) {
        // TODO: allow the caller to force rotation preference.
        if (Experiments.instance.enabled(Experiment.SnapPreferRotation))
            return preferRotation(target);
        return proportional(target);
    }

    /** Maintain translation and rotation proportionality. */
    FieldRelativeVelocity proportional(FieldRelativeVelocity speeds) {
        double maxV = m_limits.getMaxDriveVelocityM_S();
        double maxOmega = m_limits.getMaxAngleSpeedRad_S();
        double xySpeed = speeds.norm();
        double xyAngle = Math.atan2(speeds.y(), speeds.x());
        // this could be negative if xySpeed is too high
        double omegaForSpeed = maxOmega * (1 - xySpeed / maxV);
        if (Math.abs(speeds.theta()) <= omegaForSpeed) {
            // omega + xyspeed is feasible
            return speeds;
        }
        if (xySpeed < 1e-12) {
            // if we got here then omega alone is infeasible so use maxomega
            return new FieldRelativeVelocity(0, 0, Math.signum(speeds.theta()) * maxOmega);
        }
        if (Math.abs(speeds.theta()) < 1e-12) {
            // if we got here then xyspeed alone is infeasible so use maxV
            return new FieldRelativeVelocity(maxV * Math.cos(xyAngle), maxV * Math.sin(xyAngle), 0);
        }

        double v = maxOmega * xySpeed * maxV / (maxOmega * xySpeed + Math.abs(speeds.theta()) * maxV);

        double vRatio = v / xySpeed;

        return new FieldRelativeVelocity(
                vRatio * speeds.x(),
                vRatio * speeds.y(),
                vRatio * speeds.theta());
    }

    /** Scales translation to accommodate the rotation. */
    FieldRelativeVelocity preferRotation(FieldRelativeVelocity speeds) {
        double oRatio = Math.min(1, speeds.theta() / m_limits.getMaxAngleSpeedRad_S());
        double xySpeed = speeds.norm();
        double maxV = m_limits.getMaxDriveVelocityM_S();
        double xyRatio = Math.min(1, xySpeed / maxV);
        double ratio = Math.min(1 - oRatio, xyRatio);
        double xyAngle = Math.atan2(speeds.y(), speeds.x());

        return new FieldRelativeVelocity(
                ratio * maxV * Math.cos(xyAngle),
                ratio * maxV * Math.sin(xyAngle),
                speeds.theta());
    }

}
