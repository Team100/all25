package org.team100.lib.path;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.spline.HolonomicSpline;
import org.team100.lib.spline.SplineUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class PathFactory {

    public static Path100 pathFromWaypoints(
            List<HolonomicPose2d> waypoints,
            double maxDx,
            double maxDy,
            double maxDTheta,
            final List<Double> mN) {
        List<HolonomicSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(
                    new HolonomicSpline(
                            waypoints.get(i - 1),
                            waypoints.get(i),
                            mN.get(i-1),
                            mN.get(i)));
        }
        // does not force C1, theta responds too much
        // SplineUtil.forceC1(splines);
        SplineUtil.optimizeSpline(splines);
        return new Path100(PathFactory.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
    }

    public static Path100 pathFromWaypoints(
            List<HolonomicPose2d> waypoints,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        List<HolonomicSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new HolonomicSpline(waypoints.get(i - 1), waypoints.get(i)));
        }
        // does not force C1, theta responds too much
        // SplineUtil.forceC1(splines);
        SplineUtil.optimizeSpline(splines);
        return new Path100(PathFactory.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
    }

    /**
     * Make a spline from points without any control points -- the spline will go
     * through the points, computing appropriate (maybe) control points to do so.
     */
    public static Path100 withoutControlPoints(
            final List<Pose2d> waypoints,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        List<HolonomicSpline> splines = new ArrayList<>(waypoints.size() - 1);
        // first make a series of straight lines, with corners at the waypoints
        for (int i = 1; i < waypoints.size(); ++i) {
            Translation2d p0 = waypoints.get(i - 1).getTranslation();
            Translation2d p1 = waypoints.get(i).getTranslation();
            Rotation2d course = p1.minus(p0).getAngle();
            splines.add(new HolonomicSpline(
                    new HolonomicPose2d(p0, waypoints.get(i - 1).getRotation(), course),
                    new HolonomicPose2d(p1, waypoints.get(i).getRotation(), course)));
        }
        // then adjust the control points to make it C1 smooth
        SplineUtil.forceC1(splines);
        // then try to make it C2 smooth
        SplineUtil.optimizeSpline(splines);
        return new Path100(PathFactory.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
    }

    /**
     * Converts a spline into a list of Pose2dWithMotion.
     * 
     * The points are chosen so that the secant line between the points is within
     * the specified tolerance (dx, dy, dtheta) of the actual spline.
     * 
     * The trajectory scheduler consumes these points, interpolating between them
     * with straight lines. It might be better to sample the spline directly.
     *
     * @param s  the spline to parametrize
     * @param t0 starting percentage of spline to parametrize
     * @param t1 ending percentage of spline to parametrize
     * @return list of Pose2dWithCurvature that approximates the original spline
     */
    static List<Pose2dWithMotion> parameterizeSpline(
            HolonomicSpline s,
            double maxDx,
            double maxDy,
            double maxDTheta,
            double t0,
            double t1) {
        List<Pose2dWithMotion> rv = new ArrayList<>();
        rv.add(s.getPose2dWithMotion(0.0));
        double dt = (t1 - t0);
        for (double t = 0; t < t1; t += dt) {
            PathFactory.getSegmentArc(s, rv, t, t + dt, maxDx, maxDy, maxDTheta);
        }
        return rv;
    }

    public static List<Pose2dWithMotion> parameterizeSplines(
            List<? extends HolonomicSpline> splines,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        List<Pose2dWithMotion> rv = new ArrayList<>();
        if (splines.isEmpty())
            return rv;
        rv.add(splines.get(0).getPose2dWithMotion(0.0));
        for (int i = 0; i < splines.size(); i++) {
            HolonomicSpline s = splines.get(i);
            List<Pose2dWithMotion> samples = parameterizeSpline(s, maxDx, maxDy, maxDTheta, 0.0, 1.0);
            samples.remove(0);
            rv.addAll(samples);
        }
        return rv;
    }

    private static void getSegmentArc(
            HolonomicSpline s,
            List<Pose2dWithMotion> rv,
            double t0,
            double t1,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        Pose2d p0 = s.getPose2d(t0);
        Pose2d phalf = s.getPose2d(t0 + (t1 - t0) * .5);
        Pose2d p1 = s.getPose2d(t1);
        Twist2d twist_full = Pose2d.kZero.log(GeometryUtil.transformBy(GeometryUtil.inverse(p0), p1));
        Pose2d phalf_predicted = GeometryUtil.transformBy(p0,
                Pose2d.kZero.exp(GeometryUtil.scale(twist_full, 0.5)));
        Pose2d error = GeometryUtil.transformBy(GeometryUtil.inverse(phalf), phalf_predicted);

        if (GeometryUtil.norm(twist_full) < 1e-6) {
            // the Rotation2d below will be garbage in this case so give up.
            return;
        }
        Rotation2d course_predicted = (new Rotation2d(twist_full.dx, twist_full.dy))
                .rotateBy(phalf_predicted.getRotation());

        Rotation2d course_half = s.getCourse(t0 + (t1 - t0) * .5).orElse(course_predicted);
        double course_error = course_predicted.unaryMinus().rotateBy(course_half).getRadians();
        if (Math.abs(error.getTranslation().getY()) > maxDy ||
                Math.abs(error.getTranslation().getX()) > maxDx ||
                Math.abs(error.getRotation().getRadians()) > maxDTheta ||
                Math.abs(course_error) > maxDTheta) {
            getSegmentArc(s, rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
            getSegmentArc(s, rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
        } else {
            rv.add(s.getPose2dWithMotion(t1));
        }
    }
}
