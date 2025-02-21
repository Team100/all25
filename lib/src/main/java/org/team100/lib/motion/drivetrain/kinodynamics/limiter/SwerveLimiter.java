package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import java.util.function.DoubleSupplier;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Util;

/**
 * A field-relative version of the setpoint generator.
 * 
 * The robot-relative setpoint generator makes veering correction difficult, and
 * it seems unnecessary, since all our controls are field-relative.
 * 
 * Keeps the current setpoint, to avoid round-tripping through the pose
 * estimator. Remember to update the setpoint!
 */
public class SwerveLimiter {
    private final FieldRelativeVelocityLimiter m_velocityLimiter;
    private final FieldRelativeCapsizeLimiter m_capsizeLimiter;
    private final FieldRelativeAccelerationLimiter m_accelerationLimiter;
    private final SwerveDeadband m_deadband;
    private final JerkLimiter m_jerkLimiter;

    // the velocity expected at the current time step, i.e. the previous time step's
    // desire.
    private FieldRelativeVelocity m_current;
    // the desire before that, used to calculate jerk.
    private FieldRelativeVelocity m_prev;

    public SwerveLimiter(SwerveKinodynamics dynamics, DoubleSupplier voltage) {
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(dynamics, voltage);
        m_velocityLimiter = new FieldRelativeVelocityLimiter(limit);
        m_capsizeLimiter = new FieldRelativeCapsizeLimiter(dynamics);
        m_accelerationLimiter = new FieldRelativeAccelerationLimiter(dynamics);
        m_deadband = new SwerveDeadband();
        m_jerkLimiter = new JerkLimiter();
    }

    /**
     * Find a feasible setpoint in the direction of the target, and remember it for
     * next time.
     */
    public FieldRelativeVelocity apply(FieldRelativeVelocity next) {
        if (m_current == null) {
            Util.warn("no current setpoint!  using the next reference!  fix me!");
            m_current = next;
        }
        if (m_prev == null) {
            Util.warn("no previous setpoint!  using the next reference!  fix me!");
            m_prev = next;
        }
        // first limit the goal to a feasible velocity
        FieldRelativeVelocity result = m_velocityLimiter.limit(next);

        // then limit acceleration towards that goal to avoid capsize
        result = m_capsizeLimiter.limit(m_current, result);

        // finally limit acceleration further, using motor physics
        result = m_accelerationLimiter.limit(m_current, result);

        // NEW! Limit jerk if speed is close to zero.
        if (Experiments.instance.enabled(Experiment.SwerveJerkLimit)) {
            result = m_jerkLimiter.apply(m_prev, m_current, result);
        }

        // NEW! Ignore very small inputs.
        if (Experiments.instance.enabled(Experiment.SwerveDeadband)) {
            result = m_deadband.apply(result);
        }

        m_current = result;
        return m_current;
    }

    public void updateSetpoint(FieldRelativeVelocity setpoint) {
        m_current = setpoint;
    }

}
