package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.swerve.SwerveUtil;

public class FieldRelativeAccelerationLimiter {

    private final SwerveKinodynamics m_limits;

    public FieldRelativeAccelerationLimiter(SwerveKinodynamics limits) {
        m_limits = limits;
    }

    public FieldRelativeVelocity limit(
            FieldRelativeVelocity prev,
            FieldRelativeVelocity target) {
        // the accel required to achieve the target
        FieldRelativeAcceleration accel = target.accel(
                prev,
                TimedRobot100.LOOP_PERIOD_S);
        double a = accel.norm();
        double accelLimit = SwerveUtil.getAccelLimit(m_limits, prev, target);
        double scale = Math.min(1, accelLimit / a);
        return prev.plus(accel.times(scale).integrate(TimedRobot100.LOOP_PERIOD_S));

    }
}
