package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedCapsizeLimiter {
    private final SwerveKinodynamics m_limits;

    public ChassisSpeedCapsizeLimiter(SwerveKinodynamics m_limits) {
        this.m_limits = m_limits;
    }

    public ChassisSpeeds limit(
            ChassisSpeeds prev,
            ChassisSpeeds target) {
        ChassisSpeeds delta = target.minus(prev);
        double dv = GeometryUtil.norm(delta);
        double a = dv / TimedRobot100.LOOP_PERIOD_S;
        if (Math.abs(a) < 1e-6) {
            // zero accel
            return target;
        }
        double scale = Math.min(1, m_limits.getMaxCapsizeAccelM_S2() / a);
        return prev.plus(delta.times(scale));
    }
}
