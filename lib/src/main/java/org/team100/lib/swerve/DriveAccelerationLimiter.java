package org.team100.lib.swerve;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Math100;

/**
 * Enforces drive motor torque constraints.
 * 
 * This limits both acceleration (assuming a current limit) and speed (using
 * back EMF), so there's no need for a separate speed limiter.
 */
public class DriveAccelerationLimiter implements Glassy {
    private static final int kMaxIterations = 10;

    private final SwerveKinodynamics m_limits;


    public DriveAccelerationLimiter(SwerveKinodynamics limits) {
        m_limits = limits;
    }

    public double enforceWheelAccelLimit(
            double[] prev_vx,
            double[] prev_vy,
            double[] desired_vx,
            double[] desired_vy) {
        double min_s = 1.0;
        for (int i = 0; i < prev_vx.length; ++i) {
            double max_vel_step = SwerveUtil.getMaxVelStep(
                    m_limits,
                    prev_vx[i],
                    prev_vy[i],
                    desired_vx[i],
                    desired_vy[i]);


            double vx_min_s = Math100.interpolate(prev_vx[i], desired_vx[i], min_s);
            double vy_min_s = Math100.interpolate(prev_vy[i], desired_vy[i], min_s);

            double wheel_s = SwerveUtil.findDriveMaxS(
                    prev_vx[i],
                    prev_vy[i],
                    vx_min_s,
                    vy_min_s,
                    max_vel_step,
                    kMaxIterations);
            min_s = Math.min(min_s, wheel_s);
            if (min_s == 0.0) {
                break;
            }
        }
        final double s = min_s;
        return min_s;
    }

}
