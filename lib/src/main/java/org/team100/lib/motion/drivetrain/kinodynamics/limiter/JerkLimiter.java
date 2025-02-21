package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeJerk;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

public class JerkLimiter {
    // TODO: is this a reasonable limit?
    private final double m_jerkLimitM_S3 = 25;

    public FieldRelativeVelocity apply(
            FieldRelativeVelocity prev,
            FieldRelativeVelocity current,
            FieldRelativeVelocity next) {
        FieldRelativeAcceleration accel0 = current.accel(
                prev,
                TimedRobot100.LOOP_PERIOD_S);
        FieldRelativeAcceleration accel1 = next.accel(
                current,
                TimedRobot100.LOOP_PERIOD_S);
        FieldRelativeJerk jerk = accel1.jerk(accel0, TimedRobot100.LOOP_PERIOD_S);
        double j = jerk.norm();
        if (j < m_jerkLimitM_S3)
            return next;
        double scale = Math.min(1, m_jerkLimitM_S3 / j);
        FieldRelativeAcceleration accel = accel0.plus(jerk.times(scale).integrate(TimedRobot100.LOOP_PERIOD_S));
        return current.plus(accel.integrate(TimedRobot100.LOOP_PERIOD_S));
    }
}