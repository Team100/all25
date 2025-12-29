package org.team100.lib.trajectory.path;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.PathPoint;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.path.spline.HolonomicSpline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;

public class PathFactory {
    private static final boolean DEBUG = false;
    /*
     * Maximum distance of the secant lines to the continuous spline. The resulting
     * path will have little scallops if it involves rotation. In SE(2), a constant
     * "twist" segment with rotation is a curve. If the scallops are too big, make
     * this number smaller. If the trajectories are too slow to generate, make this
     * number bigger.
     */
    private static final double SPLINE_SAMPLE_TOLERANCE_M = 0.02;
    /**
     * Maximum theta error.
     */
    private static final double SPLINE_SAMPLE_TOLERANCE_RAD = 0.2;
    /**
     * Size of steps along the path. Make this number smaller for tight curves to
     * look better. Make it bigger to make trajectories (a little) faster to
     * generate.
     */
    private static final double TRAJECTORY_STEP_M = 0.1;

    private final double maxNorm;
    private final double maxDx;
    private final double maxDy;
    private final double maxDTheta;

    public PathFactory() {
        this(TRAJECTORY_STEP_M,
                SPLINE_SAMPLE_TOLERANCE_M,
                SPLINE_SAMPLE_TOLERANCE_M,
                SPLINE_SAMPLE_TOLERANCE_RAD);
    }

    public PathFactory(
            double maxNorm,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        this.maxNorm = maxNorm;
        this.maxDx = maxDx;
        this.maxDy = maxDy;
        this.maxDTheta = maxDTheta;
    }

    /**
     * A path that passes through the waypoints and control directions.
     */
    public Path100 fromWaypoints(List<WaypointSE2> waypoints) {
        List<HolonomicSpline> splines = splinesFromWaypoints(waypoints);
        return fromSplines(splines);
    }

    /////////////////////////////////////////////////////////////////////////////////////
    ///
    ///
    ///

    /**
     * Make a list of splines with the waypoints as knots.
     */
    private List<HolonomicSpline> splinesFromWaypoints(List<WaypointSE2> waypoints) {
        List<HolonomicSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new HolonomicSpline(waypoints.get(i - 1), waypoints.get(i)));
        }
        return splines;
    }

    /**
     * Converts a spline into a list of PathPoint.
     * 
     * The points are chosen so that the secant line between the points is within
     * the specified tolerance (dx, dy, dtheta) of the actual spline.
     * 
     * The trajectory scheduler consumes these points, interpolating between them
     * with straight lines.
     */
    List<PathPoint> samplesFromSpline(HolonomicSpline spline) {
        List<PathPoint> result = new ArrayList<>();
        result.add(spline.getPathPoint(0.0));
        getSegmentArc(spline, result, 0, 1);
        return result;
    }

    public Path100 fromSplines(List<? extends HolonomicSpline> splines) {
        return new Path100(samplesFromSplines(splines));
    }

    /**
     * For testing only. Do not call this directly
     */
    public List<PathPoint> samplesFromSplines(List<? extends HolonomicSpline> splines) {
        List<PathPoint> result = new ArrayList<>();
        if (splines.isEmpty())
            return result;
        result.add(splines.get(0).getPathPoint(0.0));
        for (int i = 0; i < splines.size(); i++) {
            HolonomicSpline s = splines.get(i);
            if (DEBUG)
                System.out.printf("SPLINE:\n%d\n%s\n", i, s);
            List<PathPoint> samples = samplesFromSpline(s);
            // the sample at the end of the previous spline is the same as the one for the
            // beginning of the next, so don't include it twice.
            samples.remove(0);
            result.addAll(samples);
        }
        return result;
    }

    /**
     * Recursive bisection to find a series of secant lines close to the real curve,
     * and with the points closer than maxNorm to each other, measured in L2 norm
     * (i.e. x, y, heading), and also course.
     * 
     * Note if the path is s-shaped, then bisection can find the middle :-)
     */
    private void getSegmentArc(
            HolonomicSpline spline,
            List<PathPoint> rv,
            double s0,
            double s1) {
        Pose2d p0 = spline.getPathPoint(s0).waypoint().pose();
        double shalf = (s0 + s1) / 2;
        Pose2d phalf = spline.getPathPoint(shalf).waypoint().pose();
        Pose2d p1 = spline.getPathPoint(s1).waypoint().pose();

        // twist from p0 to p1
        Twist2d twist_full = p0.log(p1);
        // twist halfway from p0 to p1
        Twist2d twist_half = GeometryUtil.scale(twist_full, 0.5);
        // point halfway from p0 to p1
        Pose2d phalf_predicted = p0.exp(twist_half);
        // difference between twist and sample
        Transform2d error = phalf_predicted.minus(phalf);

        // also prohibit large changes in direction between points
        PathPoint p20 = spline.getPathPoint(s0);
        PathPoint p21 = spline.getPathPoint(s1);
        Twist2d p2t = p20.waypoint().course().minus(p21.waypoint().course());

        // note the extra conditions to avoid points too far apart.
        // checks both translational and l2 norms
        // also checks change in course
        if (Math.abs(error.getTranslation().getX()) > maxDx
                || Math.abs(error.getTranslation().getY()) > maxDy
                || Math.abs(error.getRotation().getRadians()) > maxDTheta
                || Metrics.translationalNorm(twist_full) > maxNorm
                || Metrics.l2Norm(twist_full) > maxNorm
                || Metrics.l2Norm(p2t) > maxNorm) {
            // add a point in between
            getSegmentArc(spline, rv, s0, shalf);
            getSegmentArc(spline, rv, shalf, s1);
        } else {
            // midpoint is close enough, so add the endpoint
            rv.add(spline.getPathPoint(s1));
        }
    }
}
