package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Util;

public class FieldRelativeAccelerationLimiter {
    private static final boolean DEBUG = false;

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
        if (Math.abs(a) < 1e-6) {
            // avoid divide-by-zero
            return target;
        }
        double accelLimit = SwerveUtil.getAccelLimit(m_limits, prev, target);
        // at full speed both a and accelLimit are around zero
        if (a < accelLimit) {
            return target;
        }
        double scale = Math.min(1, accelLimit / a);
        FieldRelativeVelocity result = prev.plus(accel.times(scale).integrate(TimedRobot100.LOOP_PERIOD_S));
        if (DEBUG)
            Util.printf("accel limit prev %s target %s a %f scale %.5f %s\n", prev, target, a, scale, result);
        return result;

    }
}
