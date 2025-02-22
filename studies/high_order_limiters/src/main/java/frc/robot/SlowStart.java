package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.Util;

/**
 * A special acceleration limiter aimed at easing the launch from a stop, to
 * give the modules time to align themselves if required.
 * 
 * The modules are not particularly fast, maybe 200 ms required in the worst
 * case.
 * 
 * The error induced is the sin of the misalignment, so maybe 100 ms is enough.
 * 
 * I've measured the acceleration rate and found pretty low values, like a few
 * m/s/s, so a reasonable velocity ceiling for the "launch" period might be 0.1
 * m/s.
 * 
 * TODO: adjust these parameters.
 */
public class SlowStart {
    /** The velocity threshold to apply the limit */
    private final double m_threshold = 0.1;
    /** The allowed accel at low speed */
    private final double m_limit = .5;

    public FieldRelativeVelocity limit(
            FieldRelativeVelocity prev,
            FieldRelativeVelocity target) {
        if (prev.norm() > m_threshold)
            return target;
        // TODO: reduce the duplication with the accel limiter.
        FieldRelativeAcceleration accel = target.accel(
                prev,
                TimedRobot100.LOOP_PERIOD_S);
        double a = accel.norm();
        if (Math.abs(a) < 1e-6) {
            // avoid divide-by-zero
            return target;
        }
        if (a < m_limit) {
            return target;
        }
        double scale = Math.min(1, m_limit / a);
        FieldRelativeVelocity result = prev.plus(accel.times(scale).integrate(TimedRobot100.LOOP_PERIOD_S));
        return result;
    }
}
