package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

public class FieldRelativeCapsizeLimiter {
    private final SwerveKinodynamics m_limits;

    public FieldRelativeCapsizeLimiter(SwerveKinodynamics m_limits) {
        this.m_limits = m_limits;
    }

    public FieldRelativeVelocity limit(
            FieldRelativeVelocity prev,
            FieldRelativeVelocity target) {
        FieldRelativeAcceleration accel = target.accel(
                prev,
                TimedRobot100.LOOP_PERIOD_S);
        double a = accel.norm();
        if (a < 1e-6) {
            // zero accel
            return target;
        }
        double scale = Math.min(1, m_limits.getMaxCapsizeAccelM_S2() / a);
        return prev.plus(accel.times(scale).integrate(TimedRobot100.LOOP_PERIOD_S));
    }
}
