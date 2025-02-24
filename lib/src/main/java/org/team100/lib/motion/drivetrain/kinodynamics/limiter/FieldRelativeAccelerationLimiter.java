package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Util;

public class FieldRelativeAccelerationLimiter {
    private static final boolean DEBUG = false;
    private final SwerveKinodynamics m_limits;
    private final double m_alphaScale;

    public FieldRelativeAccelerationLimiter(SwerveKinodynamics limits, double alphaScale) {
        m_limits = limits;
        m_alphaScale = alphaScale;
    }

    public FieldRelativeVelocity limit(
            FieldRelativeVelocity prev,
            FieldRelativeVelocity target) {
        // the accel required to achieve the target
        FieldRelativeAcceleration accel = target.accel(
                prev,
                TimedRobot100.LOOP_PERIOD_S);
        double scale = Math.min(cartesianScale(prev, target, accel), alphaScale(accel));

        FieldRelativeVelocity result = prev.plus(accel.times(scale).integrate(TimedRobot100.LOOP_PERIOD_S));
        if (DEBUG)
            Util.printf("accel limit prev %s target %s scale %.5f %s\n", prev, target, scale, result);
        return result;

    }

    private double cartesianScale(
            FieldRelativeVelocity prev,
            FieldRelativeVelocity target,
            FieldRelativeAcceleration accel) {
        double a = accel.norm();
        if (Math.abs(a) < 1e-6) {
            // avoid divide-by-zero
            return 1.0;
        }
        double accelLimit = SwerveUtil.getAccelLimit(m_limits, prev, target);
        // at full speed both a and accelLimit are around zero
        if (a < accelLimit) {
            return 1.0;
        }
        return Math.min(1, accelLimit / a);
    }

    private double alphaScale(FieldRelativeAcceleration accel) {
        double a = accel.theta();
        if (Math.abs(a) < 1e-6) {
            // avoid divide-by-zero
            return 1.0;
        }
        double accelLimit = m_alphaScale * m_limits.getMaxAngleAccelRad_S2();
        if (a < accelLimit) {
            return 1.0;
        }
        return Math.min(1, accelLimit / a);

    }

}
