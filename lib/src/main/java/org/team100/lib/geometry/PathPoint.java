package org.team100.lib.geometry;

import org.team100.lib.trajectory.path.spline.HolonomicSpline;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;

/**
 * Represents a point on a path in SE(2).
 * 
 * Includes a WaypointSE2, heading rate, and curvature.
 */
public class PathPoint {
    private static final boolean DEBUG = false;
    /** Pose and course. */
    private final WaypointSE2 m_waypoint;
    /** The source of this point (for resampling) */
    private final HolonomicSpline m_spline;
    /** The parameter value of this point (for resampling) */
    private final double m_s;
    /** Change in heading per meter of motion, rad/m. */
    private final double m_headingRateRad_M;
    /** Change in course per change in distance, rad/m. */
    private final double m_curvatureRad_M;

    /**
     * @param waypoint         location and heading and direction of travel
     * @param headingRateRad_M change in heading, per meter traveled
     * @param curvatureRad_M   change in course per meter traveled.
     */
    public PathPoint(
            WaypointSE2 waypoint,
            HolonomicSpline spline,
            double s,
            double headingRateRad_M,
            double curvatureRad_M) {
        m_waypoint = waypoint;
        m_spline = spline;
        m_s = s;
        m_headingRateRad_M = headingRateRad_M;
        m_curvatureRad_M = curvatureRad_M;
    }

    public PathPoint(
            WaypointSE2 waypoint,
            double headingRateRad_M,
            double curvatureRad_M) {
        m_waypoint = waypoint;
        m_spline = null;
        m_s = 0;
        m_headingRateRad_M = headingRateRad_M;
        m_curvatureRad_M = curvatureRad_M;
    }

    public WaypointSE2 waypoint() {
        return m_waypoint;
    }

    /**
     * Heading rate is radians per meter.
     * 
     * If you want radians per second, multiply by velocity (meters per second).
     */
    public double getHeadingRateRad_M() {
        return m_headingRateRad_M;
    }

    /** Radians per meter, which is the reciprocal of the radius. */
    public double getCurvatureRad_M() {
        return m_curvatureRad_M;
    }

    public double getS() {
        return m_s;
    }

    /**
     * Linear interpolation of each component separately.
     * 
     * TODO: this is wrong for the spline parameter, it's the distance.
     * 
     * Not a constant-twist arc.
     */
    public PathPoint interpolate(PathPoint other, double x) {
        if (DEBUG)
            System.out.printf("this s %f other s %f\n",
                    m_s, other.m_s);
        HolonomicSpline spline = null;
        double s = 0;
        if (m_spline == other.m_spline) {
            // ok to interpolate using this spline
            if (DEBUG)
                System.out.println("same spline");
            spline = m_spline;
            s = Math100.interpolate(m_s, other.m_s, x);
        } else {
            // which one to use?
            // one of the endpoints should be the spline endpoint
            // which is always the zero (not the 1)
            if (other.m_s < 1e-6) {
                // other one is the start, so use this one
                if (DEBUG)
                    System.out.println("use this spline");
                spline = m_spline;
                s = Math100.interpolate(m_s, 1, x);
            } else {
                if (DEBUG)
                    System.out.println("use the other spline");
                spline = other.m_spline;
                s = Math100.interpolate(0, other.m_s, x);
            }
        }
        if (DEBUG)
            System.out.printf("s0 %f s1 %f x %f s %f\n",
                    m_s, other.m_s, x, s);
        // sample the spline again instead of interpolating.
        if (spline != null)
            return spline.getPathPoint(s);
        // TODO: remove this way
        System.out.println("WARNING: no spline, using linear interpolation ");
        return new PathPoint(
                GeometryUtil.interpolate(m_waypoint, other.m_waypoint, x),
                spline,
                s,
                MathUtil.interpolate(m_headingRateRad_M, other.m_headingRateRad_M, x),
                Math100.interpolate(m_curvatureRad_M, other.m_curvatureRad_M, x));
    }

    /**
     * R2 (xy) planar distance only (IGNORES ROTATION) so that planar
     * velocity and curvature works correctly. Not the twist arclength.
     * Not the double-geodesic L2 thing. Just XY translation hypot.
     * 
     * Always non-negative.
     */
    public double distanceCartesian(PathPoint other) {
        return Metrics.translationalDistance(m_waypoint.pose(), other.m_waypoint.pose());
    }

    public boolean equals(Object other) {
        if (!(other instanceof PathPoint)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }

        PathPoint p2dwc = (PathPoint) other;
        if (!m_waypoint.equals(p2dwc.m_waypoint)) {
            if (DEBUG)
                System.out.println("wrong waypoint");
            return false;
        }
        if (!Math100.epsilonEquals(m_headingRateRad_M, p2dwc.m_headingRateRad_M)) {
            if (DEBUG)
                System.out.println("wrong heading rate");
            return false;
        }
        if (!Math100.epsilonEquals(m_curvatureRad_M, p2dwc.m_curvatureRad_M)) {
            if (DEBUG)
                System.out.println("wrong curvature");
            return false;
        }
        return true;
    }

    public String toString() {
        return String.format(
                "x %5.3f, y %5.3f, theta %5.3f, course %s, dtheta %5.3f, curvature %5.3f",
                m_waypoint.pose().getTranslation().getX(),
                m_waypoint.pose().getTranslation().getY(),
                m_waypoint.pose().getRotation().getRadians(),
                m_waypoint.course(),
                m_headingRateRad_M,
                m_curvatureRad_M);
    }

}