package org.team100.lib.path;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.spline.HolonomicSpline;
import org.team100.lib.spline.SplineGenerator;
import org.team100.lib.spline.SplineUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathPlanner {

    public static Path100 pathFromWaypointsAndHeadings(
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            double maxDx,
            double maxDy,
            double maxDTheta,
            final List<Double> mN) {
        List<HolonomicSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new HolonomicSpline(
                    waypoints.get(i - 1), waypoints.get(i),
                    headings.get(i - 1), headings.get(i), mN.get(i)));
        }
        // does not force C1, theta responds too much
        // SplineUtil.forceC1(splines);
        SplineUtil.optimizeSpline(splines);
        return new Path100(SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
    }

    public static Path100 pathFromWaypointsAndHeadings(
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        List<HolonomicSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new HolonomicSpline(
                    waypoints.get(i - 1), waypoints.get(i),
                    headings.get(i - 1), headings.get(i)));
        }
        // does not force C1, theta responds too much
        // SplineUtil.forceC1(splines);
        SplineUtil.optimizeSpline(splines);
        return new Path100(SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
    }

    /**
     * Make a spline from points without any control points -- the spline will go
     * through the points, computing appropriate (maybe) control points to do so.
     */
    public static Path100 withoutControlPoints(
            final List<Translation2d> waypoints,
            final List<Rotation2d> headings,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        List<HolonomicSpline> splines = new ArrayList<>(waypoints.size() - 1);
        // first make a series of straight lines, with corners at the waypoints
        for (int i = 1; i < waypoints.size(); ++i) {
            Translation2d p0 = waypoints.get(i - 1);
            Translation2d p1 = waypoints.get(i);
            Rotation2d course = p1.minus(p0).getAngle();
            splines.add(new HolonomicSpline(
                    new Pose2d(p0, course), new Pose2d(p1, course),
                    headings.get(i - 1), headings.get(i)));
        }
        // then adjust the control points to make it C1 smooth
        SplineUtil.forceC1(splines);
        // then try to make it C2 smooth
        SplineUtil.optimizeSpline(splines);
        return new Path100(SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
    }
}
