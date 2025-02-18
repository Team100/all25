package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import java.util.function.DoubleSupplier;

import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

/**
 * A field-relative version of the setpoint generator.
 * 
 * The robot-relative setpoint generator makes veering correction difficult, and
 * it seems unnecessary, since all our controls are field-relative.
 */
public class SwerveLimiter {
    private final FieldRelativeVelocityLimiter m_velocityLimiter;
    private final FieldRelativeCapsizeLimiter m_capsizeLimiter;
    private final FieldRelativeAccelerationLimiter m_accelerationLimiter;

    public SwerveLimiter(SwerveKinodynamics dynamics, DoubleSupplier voltage) {
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(dynamics, voltage);
        m_velocityLimiter = new FieldRelativeVelocityLimiter(limit);
        m_capsizeLimiter = new FieldRelativeCapsizeLimiter(dynamics);
        m_accelerationLimiter = new FieldRelativeAccelerationLimiter(dynamics);
    }

    public FieldRelativeVelocity apply(FieldRelativeVelocity current, FieldRelativeVelocity next) {
        // first limit the goal to a feasible velocity
        FieldRelativeVelocity result = m_velocityLimiter.limit(next);
        // then limit acceleration towards that goal to avoid capsize
        result = m_capsizeLimiter.limit(current, result);
        // finally limit acceleration further, using motor physics
        return m_accelerationLimiter.limit(current, result);
    }

}
