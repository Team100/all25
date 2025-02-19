package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Math100;

public class SwerveUtil {

    /**
     * At low speed, accel is limited by the current limiters.
     * At high speed, accel is limited by back EMF.
     * Deceleration limits are different: back EMF is helping in that case.
     * 
     * @see SwerveDriveDynamicsConstraint.getMinMaxAcceleration().
     */
    public static double getAccelLimit(
            SwerveKinodynamics m_limits,
            FieldRelativeVelocity prev,
            FieldRelativeVelocity desired) {
        if (isAccel(prev, desired)) {
            return minAccel(m_limits, prev.norm());
        }
        return m_limits.getMaxDriveDecelerationM_S2();
    }

    /**
     * At low speed, accel is limited by the current limiters.
     * At high speed, accel is limited by back EMF.
     */
    public static double minAccel(SwerveKinodynamics m_limits, double velocity) {
        double speedFraction = Math100.limit(velocity / m_limits.getMaxDriveVelocityM_S(), 0, 1);
        double backEmfLimit = 1 - speedFraction;
        double backEmfLimitedAcceleration = backEmfLimit * m_limits.getStallAccelerationM_S2();
        double currentLimitedAcceleration = m_limits.getMaxDriveAccelerationM_S2();
        return Math.min(backEmfLimitedAcceleration, currentLimitedAcceleration);
    }

    /**
     * Find the desired dv. Project it on to the previous v: if the projection is
     * positive, we're accelerating, otherwise decelerating.
     * 
     * This correctly captures sharp turns as decelerations; simply comparing the
     * magnitudes of initial and final velocities is not correct.
     */
    static boolean isAccel(FieldRelativeVelocity prev,
            FieldRelativeVelocity target) {
        FieldRelativeAcceleration accel = target.accel(prev, TimedRobot100.LOOP_PERIOD_S);
        double dot = prev.x() * accel.x() + prev.y() * accel.y();
        return dot >= 0;
    }

    private SwerveUtil() {
        //
    }
}
