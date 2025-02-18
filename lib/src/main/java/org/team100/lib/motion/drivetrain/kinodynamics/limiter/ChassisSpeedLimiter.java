package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Return feasible speeds, using a simple worst-case model (diagonal course).
 */
public class ChassisSpeedLimiter {
    private final BatterySagSpeedLimit m_limits;

    public ChassisSpeedLimiter(BatterySagSpeedLimit limit) {
        m_limits = limit;
    }

    public ChassisSpeeds limit(ChassisSpeeds target) {
        if (Experiments.instance.enabled(Experiment.SnapPreferRotation))
            return preferRotation(target);
        return proportional(target);
    }

    /**
     * Maintain translation and rotation proportionality.
     */
    ChassisSpeeds proportional(ChassisSpeeds speeds) {
        double maxV = m_limits.getMaxDriveVelocityM_S();
        double maxOmega = m_limits.getMaxAngleSpeedRad_S();
        double xySpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double xyAngle = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);
        // this could be negative if xySpeed is too high
        double omegaForSpeed = maxOmega * (1 - xySpeed / maxV);
        if (Math.abs(speeds.omegaRadiansPerSecond) <= omegaForSpeed) {
            // omega + xyspeed is feasible
            return speeds;
        }
        if (xySpeed < 1e-12) {
            // if we got here then omega alone is infeasible so use maxomega
            return new ChassisSpeeds(0, 0, Math.signum(speeds.omegaRadiansPerSecond) * maxOmega);
        }
        if (Math.abs(speeds.omegaRadiansPerSecond) < 1e-12) {
            // if we got here then xyspeed alone is infeasible so use maxV
            return new ChassisSpeeds(maxV * Math.cos(xyAngle), maxV * Math.sin(xyAngle), 0);
        }

        double v = maxOmega * xySpeed * maxV / (maxOmega * xySpeed + Math.abs(speeds.omegaRadiansPerSecond) * maxV);

        double vRatio = v / xySpeed;

        return new ChassisSpeeds(
                vRatio * speeds.vxMetersPerSecond,
                vRatio * speeds.vyMetersPerSecond,
                vRatio * speeds.omegaRadiansPerSecond);
    }

    /** Scales translation to accommodate the rotation. */
    ChassisSpeeds preferRotation(ChassisSpeeds speeds) {
        double oRatio = Math.min(1, speeds.omegaRadiansPerSecond / m_limits.getMaxAngleSpeedRad_S());
        double xySpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double maxV = m_limits.getMaxDriveVelocityM_S();
        double xyRatio = Math.min(1, xySpeed / maxV);
        double ratio = Math.min(1 - oRatio, xyRatio);
        double xyAngle = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);

        return new ChassisSpeeds(
                ratio * maxV * Math.cos(xyAngle),
                ratio * maxV * Math.sin(xyAngle),
                speeds.omegaRadiansPerSecond);
    }

}
