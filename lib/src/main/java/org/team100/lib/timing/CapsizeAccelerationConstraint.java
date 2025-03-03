package org.team100.lib.timing;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

/**
 * Velocity limit based on curvature and the capsize limit (scaled).
 */
public class CapsizeAccelerationConstraint implements TimingConstraint {
    private final SwerveKinodynamics m_limits;
    private final double m_scale;

    /**
     * Use the factory.
     * 
     * @param limits absolute maxima
     * @param scale  apply to the maximum capsize accel to get the actual
     *               constraint. this is useful to slow down trajectories in
     *               sharp curves, which makes odometry more accurate and reduces
     *               the effect of steering lag.
     */
    public CapsizeAccelerationConstraint(SwerveKinodynamics limits, double scale) {
        m_limits = limits;
        m_scale = scale;
    }

    /**
     * The centripetal acceleration as a function of linear speed and radius:
     * a = v^2 / r
     * so
     * v = sqrt(a * r)
     * If the curvature is zero, this will return infinity.
     */
    @Override
    public NonNegativeDouble getMaxVelocity(final Pose2dWithMotion state) {
        double mMaxCentripetalAccel = m_limits.getMaxCapsizeAccelM_S2() * m_scale;
        double radius = 1 / state.getCurvature();
        // abs is used here to make sure sqrt is happy.
        return new NonNegativeDouble(Math.sqrt(Math.abs(mMaxCentripetalAccel * radius)));
    }

    /**
     * Acceleration has two components: along the path (which is what is returned
     * here), and across the path. Assuming the robot is "round," i.e. the capsize
     * limit is the same in all directions, then it's the *total* acceleration that
     * should be limited. This returns the along-the-path component of that total.
     * 
     * centripetal = v^2 / r
     * total^2 = centripetal^2 + along^2
     * so
     * along = sqrt(total^2 - v^4/r^2)
     */
    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocity) {
        double mMaxCentripetalAccel = m_limits.getMaxCapsizeAccelM_S2() * m_scale;

        double radius = 1 / state.getCurvature();
        double centripetalAccel = velocity * velocity / radius;
        double alongsq = mMaxCentripetalAccel * mMaxCentripetalAccel - centripetalAccel * centripetalAccel;
        if (alongsq < 0) {
            // if you're here, you're violating the velocity constraint above,
            // and you should try to gently slow down.
            return new MinMaxAcceleration(-m_limits.getMaxDriveDecelerationM_S2() * m_scale, 0);
        }
        double along = Math.sqrt(alongsq);
        return new MinMaxAcceleration(-along, along);
    }
}
