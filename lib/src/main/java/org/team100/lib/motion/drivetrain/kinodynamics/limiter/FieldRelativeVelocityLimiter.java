package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.Util;

/**
 * Return feasible velocity, using a simple worst-case model (diagonal course).
 */
public class FieldRelativeVelocityLimiter implements Glassy {
    private static final boolean DEBUG = false;

    private final DoubleLogger m_log_scale;
    private final BatterySagSpeedLimit m_limits;

    public FieldRelativeVelocityLimiter(
            LoggerFactory parent,
            BatterySagSpeedLimit limit) {
        LoggerFactory child = parent.child(this);
        m_log_scale = child.doubleLogger(Level.TRACE, "scale");
        m_limits = limit;
    }

    public FieldRelativeVelocity apply(FieldRelativeVelocity target) {
        if (Experiments.instance.enabled(Experiment.LimitsPreferRotation))
            return preferRotation(target);
        return proportional(target);
    }

    /** Maintain translation and rotation proportionality. */
    FieldRelativeVelocity proportional(FieldRelativeVelocity speeds) {
        if (DEBUG)
            Util.printf("proportional %s\n", speeds);
        double maxV = m_limits.getMaxDriveVelocityM_S();
        double maxOmega = m_limits.getMaxAngleSpeedRad_S();
        double xySpeed = speeds.norm();
        double xyAngle = Math.atan2(speeds.y(), speeds.x());
        // this could be negative if xySpeed is too high
        double omegaForSpeed = maxOmega * (1 - xySpeed / maxV);
        if (Math.abs(speeds.theta()) <= omegaForSpeed) {
            // omega + xyspeed is feasible
            m_log_scale.log(() -> 1.0);
            if (DEBUG)
                Util.printf("feasible %s\n", speeds);
            return speeds;
        }
        if (xySpeed < 1e-12) {
            // Omega alone is infeasible, so use maxOmega.
            double scale = Math.abs(maxOmega / speeds.theta());
            m_log_scale.log(() -> scale);
            if (DEBUG)
                Util.printf("max omega %s\n", speeds);
            return new FieldRelativeVelocity(0, 0, Math.signum(speeds.theta()) * maxOmega);
        }
        if (Math.abs(speeds.theta()) < 1e-12) {
            // Cartesian speed alone is infeasible, so use maxV.
            double scale = Math.abs(maxV / xySpeed);
            m_log_scale.log(() -> scale);
            if (DEBUG)
                Util.printf("max v %s\n", speeds);
            return new FieldRelativeVelocity(maxV * Math.cos(xyAngle), maxV * Math.sin(xyAngle), 0);
        }

        double v = maxOmega * xySpeed * maxV / (maxOmega * xySpeed + Math.abs(speeds.theta()) * maxV);

        double scale = v / xySpeed;

        m_log_scale.log(() -> scale);

        if (DEBUG)
            Util.printf("FieldRelativeVelocityLimiter proportional scale %.5f\n", scale);

        return new FieldRelativeVelocity(
                scale * speeds.x(),
                scale * speeds.y(),
                scale * speeds.theta());
    }

    /** Scales translation to accommodate the rotation. */
    FieldRelativeVelocity preferRotation(FieldRelativeVelocity speeds) {
        double omegaRatio = Math.min(1, speeds.theta() / m_limits.getMaxAngleSpeedRad_S());
        double xySpeed = speeds.norm();
        double maxV = m_limits.getMaxDriveVelocityM_S();
        double xyRatio = Math.min(1, xySpeed / maxV);
        double ratio = Math.min(1 - omegaRatio, xyRatio);

        // just for logging
        double scale = ratio / xyRatio;
        m_log_scale.log(() -> scale);

        double xyAngle = Math.atan2(speeds.y(), speeds.x());

        if (DEBUG)
            Util.printf("FieldRelativeVelocityLimiter rotation ratio %.5f\n", ratio);

        return new FieldRelativeVelocity(
                ratio * maxV * Math.cos(xyAngle),
                ratio * maxV * Math.sin(xyAngle),
                speeds.theta());
    }

}
