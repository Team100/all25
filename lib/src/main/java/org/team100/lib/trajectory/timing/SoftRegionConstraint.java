package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.PathPoint;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Limits acceleration near a point.
 * 
 * There's a minimum at the point, and a maximum at the edge of a circle
 * surrounding it, interpolated using the cube root of the distance.  This
 * provides roughly constant jerk (i.e. constant growth rate of acceleration).
 * 
 * To increase resolution of the slow part, you should use the TrajectoryRecycler.
 * 
 * https://docs.google.com/spreadsheets/d/1sbB-zTBUjRRlWHaWXe-V1ZDhAZCFwItVVO1x3LmZ4B4/edit?gid=691127145#gid=691127145
 */
public class SoftRegionConstraint implements TimingConstraint {
    private final Translation2d m_center;
    private final double m_radius;
    private final double m_min;
    private final double m_max;

    /**
     * @param center of the soft region
     * @param radius of the soft region
     * @param min    accel/decel at the center
     * @param max    accel/decel at the edge
     */
    public SoftRegionConstraint(
            Translation2d center, double radius, double min, double max) {
        m_center = center;
        m_radius = radius;
        m_min = min;
        m_max = max;
    }

    @Override
    public double maxV(PathPoint state) {
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public double maxAccel(PathPoint state, double velocityM_S) {
        double s = near(state);
        if (s < 1)
            return Math100.interpolate(m_min, m_max, Math.cbrt(s));
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public double maxDecel(PathPoint state, double velocityM_S) {
        double s = near(state);
        if (s < 1)
            return -1.0 * Math100.interpolate(m_min, m_max, Math.cbrt(s));
        return Double.NEGATIVE_INFINITY;
    }

    /** distance to the center compared with the radius */
    private double near(PathPoint state) {
        Translation2d translation = state.waypoint().pose().getTranslation();
        return translation.getDistance(m_center) / m_radius;
    }

}
