package org.team100.lib.trajectory.path;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.PathPoint;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Represents a 2d holonomic path, i.e. with heading independent from course.
 * 
 * There's no timing information here. For that, see Trajectory100.
 */
public class Path100 {
    private static final boolean DEBUG = false;
    // if an interpolated point is more than this far from an endpoint,
    // it indicates the endpoints are too far apart, including too far apart
    // in rotation, which is an aspect of the path scheduling that the
    // scheduler can't see
    // TODO: make this a constructor parameter.
    private static final double INTERPOLATION_LIMIT = 0.3;
    private final List<PathPoint> m_points;
    /**
     * Translational distance, just the xy plane, not the Twist arc
     * or anything else, just xy distance.
     */
    private final double[] m_distances;

    public Path100(final List<PathPoint> states) {
        int n = states.size();
        m_points = new ArrayList<>(n);
        m_distances = new double[n];
        if (states.isEmpty()) {
            return;
        }
        m_distances[0] = 0.0;
        m_points.add(states.get(0));
        for (int i = 1; i < n; ++i) {
            m_points.add(states.get(i));
            PathPoint p0 = getPoint(i - 1);
            PathPoint p1 = getPoint(i);
            double dist = Metrics.translationalDistance(p0.waypoint().pose(), p1.waypoint().pose());
            m_distances[i] = m_distances[i - 1] + dist;
        }
    }

    public boolean isEmpty() {
        return m_points.isEmpty();
    }

    public int length() {
        return m_points.size();
    }

    public PathPoint getPoint(int index) {
        if (m_points.isEmpty())
            return null;
        return m_points.get(index);
    }

    /** This is always non-negative. */
    public double getMaxDistance() {
        if (m_points.isEmpty())
            return 0.0;
        return m_distances[m_distances.length - 1];
    }

    /**
     * Walks through all the points to find the bracketing points, and then
     * interpolates between them.
     * 
     * Beware, can return null if the path is empty.
     * 
     * @param distance in meters, always a non-negative number.
     */
    public PathPoint sample(double distance) {
        if (distance >= getMaxDistance()) {
            // off the end
            return getPoint(length() - 1);
        }
        if (distance <= 0.0) {
            // before the start
            return getPoint(0);
        }
        for (int i = 1; i < length(); ++i) {
            // walk through the points to bracket the desired distance
            PathPoint p0 = getPoint(i - 1);
            PathPoint p1 = getPoint(i);
            double d0 = m_distances[i - 1];
            double d1 = m_distances[i];
            double d = d1 - d0;
            if (d1 >= distance) {
                // Found the bracket.
                double s = (distance - d0) / d;
                PathPoint lerp = p0.interpolate(p1, s);
                // disallow corners
                Twist2d t0 = p0.waypoint().course().minus(lerp.waypoint().course());
                double l0 = Metrics.l2Norm(t0);
                Twist2d t1 = p1.waypoint().course().minus(lerp.waypoint().course());
                double l1 = Metrics.l2Norm(t1);
                if (Math.max(l0, l1) > INTERPOLATION_LIMIT)
                    System.out.printf(
                            "WARNING!  Interpolated value too far away, p0=%s, p1=%s, t0=%s t1=%s.  This usually indicates a sharp corner in the path, which is not allowed.",
                            p0, p1, t0, t1);
                return lerp;
            }
        }
        return null;
    }

    /** Just returns the list of points with no further sampling. */
    public PathPoint[] resample() {
        return m_points.toArray(PathPoint[]::new);
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < length(); ++i) {
            builder.append(i);
            builder.append(": ");
            builder.append(getPoint(i));
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }

    ////////////////////////////////////////////////////////////////////
    ///
    /// DANGER ZONE
    ///
    /// Don't use anything here unless you know what you're doing.
    ///

    /**
     * Samples the entire path evenly by distance. Since the spline parameterizer
     * now contains a pathwise distance limit, you shouldn't need this anymore.
     */
    public PathPoint[] resample(double step) {
        double maxDistance = getMaxDistance();
        if (maxDistance == 0)
            throw new IllegalArgumentException("max distance must be greater than zero");
        int num_states = (int) Math.ceil(maxDistance / step) + 1;
        if (DEBUG)
            System.out.printf("resample max distance %f step %f num states %d f %f\n",
                    maxDistance, step, num_states, maxDistance / step);
        PathPoint[] samples = new PathPoint[num_states];
        for (int i = 0; i < num_states; ++i) {
            // the dtheta and curvature come from here and are never changed.
            // the values here are just interpolated from the original values.
            double d = Math.min(i * step, maxDistance);
            PathPoint state = sample(d);
            if (state == null) continue; 
            if (DEBUG)
                System.out.printf("RESAMPLE: i=%d d=%f state=%s\n", i, d, state);
            samples[i] = state;
        }
        return samples;
    }
}
