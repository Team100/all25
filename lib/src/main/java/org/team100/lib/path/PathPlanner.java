package org.team100.lib.path;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.spline.HolonomicSpline;
import org.team100.lib.spline.SplineGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
                    headings.get(i - 1), headings.get(i), mN.get(i)) );
        }
        HolonomicSpline.optimizeSpline(splines);
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
        HolonomicSpline.optimizeSpline(splines);
        return new Path100(SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
    }
    
}
