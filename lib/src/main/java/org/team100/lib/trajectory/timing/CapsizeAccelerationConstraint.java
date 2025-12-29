package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.PathPoint;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.tuning.Mutable;

/**
 * Velocity limit based on curvature and the capsize limit (scaled).
 */
public class CapsizeAccelerationConstraint implements TimingConstraint {
    private static final boolean DEBUG = false;
    private final Mutable m_scale;
    private final double m_maxCentripetalAccel;
    private final double m_maxDecel;

    /**
     * Use the factory.
     * 
     * @param limits absolute maxima
     * @param scale  apply to the maximum capsize accel to get the actual
     *               constraint. this is useful to slow down trajectories in
     *               sharp curves, which makes odometry more accurate and reduces
     *               the effect of steering lag.
     */
    public CapsizeAccelerationConstraint(
            LoggerFactory parent,
            SwerveKinodynamics limits,
            double scale) {
        LoggerFactory log = parent.type(this);
        m_scale = new Mutable(log, "scale", scale);
        m_maxCentripetalAccel = limits.getMaxCapsizeAccelM_S2();
        m_maxDecel = -limits.getMaxDriveDecelerationM_S2();
    }

    public CapsizeAccelerationConstraint(LoggerFactory parent, double centripetal, double decel) {
        LoggerFactory log = parent.type(this);
        m_scale = new Mutable(log, "scale", 1);
        m_maxCentripetalAccel = centripetal;
        m_maxDecel = -decel;
    }

    /**
     * The centripetal acceleration as a function of linear speed and radius:
     * a = v^2 / r
     * so
     * v = sqrt(a * r)
     * If the curvature is zero, this will return infinity.
     */
    @Override
    public double maxV(final PathPoint state) {
        double radius = 1 / Math.abs(state.getCurvatureRad_M());
        // abs is used here to make sure sqrt is happy.
        double maxV = Math.sqrt(Math.abs(m_maxCentripetalAccel * m_scale.getAsDouble() * radius));
        if (DEBUG)
            System.out.printf("maxV %f\n", maxV);
        return maxV;
    }

    @Override
    public double maxAccel(PathPoint state, double velocity) {
        double alongsq = alongSq(state, velocity);
        if (alongsq < 0) {
            if (DEBUG)
                System.out.println("too fast for the curvature, can't speed up");
            return 0;
        }
        double maxA = Math.sqrt(alongsq);
        if (DEBUG)
            System.out.printf("maxA %f\n", maxA);
        return maxA;
    }

    @Override
    public double maxDecel(PathPoint state, double velocity) {
        double alongsq = alongSq(state, velocity);
        if (alongsq < 0) {
            if (DEBUG)
                System.out.println("too fast for the curvature, slowing down is ok");
            return m_maxDecel * m_scale.getAsDouble();
        }
        double maxD = -Math.sqrt(alongsq);
        if (DEBUG)
            System.out.printf("maxD %f\n", maxD);
        return maxD;
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
    private double alongSq(PathPoint state, double velocity) {
        double radius = 1 / Math.abs(state.getCurvatureRad_M());
        double actualCentripetalAccel = velocity * velocity / radius;
        if (DEBUG)
            System.out.printf("radius %f velocity %f actual centripetal accel %f\n",
                    radius, velocity, actualCentripetalAccel);
        return m_maxCentripetalAccel * m_scale.getAsDouble() * m_maxCentripetalAccel * m_scale.getAsDouble()
                - actualCentripetalAccel * actualCentripetalAccel;
    }
}
