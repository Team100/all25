package org.team100.lib.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.spline.HolonomicSpline;
import org.team100.lib.timing.ScheduleGenerator.TimingException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class Path100Test {
    private static final double kDelta = 0.001;

    private static final List<Rotation2d> kHeadings = Arrays.asList(
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(30),
            GeometryUtil.fromDegrees(60),
            GeometryUtil.fromDegrees(90));

    private static final List<Pose2dWithMotion> kWaypoints = Arrays.asList(
            new Pose2dWithMotion(new Pose2d(new Translation2d(0.0, 0.0), kHeadings.get(0))),
            new Pose2dWithMotion(new Pose2d(new Translation2d(24.0, 0.0), kHeadings.get(1))),
            new Pose2dWithMotion(new Pose2d(new Translation2d(36.0, 12.0), kHeadings.get(2))),
            new Pose2dWithMotion(new Pose2d(new Translation2d(60.0, 12.0), kHeadings.get(3))));

    @Test
    void testEmpty() {
        List<HolonomicSpline> splines = new ArrayList<>();
        double maxDx = 0.1;
        double maxDy = 0.1;
        double maxDTheta = 0.1;
        Path100 path = new Path100(PathFactory.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
        assertEquals(0, path.length(), 0.001);
    }

    @Test
    void testSimple() {
        // spline is in the x direction, no curvature.
        HolonomicSpline spline = new HolonomicSpline(
                new HolonomicPose2d(
                        new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(
                        new Translation2d(1, 0), new Rotation2d(), new Rotation2d())) {

            @Override
            public Translation2d getPoint(double t) {
                return new Translation2d(t, 0);
            }

            @Override
            public Rotation2d getHeading(double t) {
                return Rotation2d.kZero;
            }

            @Override
            public Optional<Rotation2d> getCourse(double t) {
                return Optional.of(Rotation2d.kZero);
            }

            @Override
            public double getDHeading(double t) {
                return 0;
            }

            @Override
            public double getCurvature(double t) {
                return 0;
            }

            @Override
            public double getDCurvature(double t) {
                return 0;
            }

            @Override
            public double getVelocity(double t) {
                return 1;
            }
        };
        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(spline);
        double maxDx = 0.1;
        double maxDy = 0.1;
        double maxDTheta = 0.1;
        Path100 path = new Path100(PathFactory.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
        assertEquals(2, path.length(), 0.001);
    }

    @Test
    void testConstruction() {
        Path100 traj = new Path100(kWaypoints);
        assertFalse(traj.isEmpty());
        assertEquals(4, traj.length());
    }

    @Test
    void testStateAccessors() {
        Path100 traj = new Path100(kWaypoints);

        assertEquals(kWaypoints.get(0), traj.getPoint(0));
        assertEquals(kWaypoints.get(1), traj.getPoint(1));
        assertEquals(kWaypoints.get(2), traj.getPoint(2));
        assertEquals(kWaypoints.get(3), traj.getPoint(3));

        assertEquals(kHeadings.get(0), traj.getPoint(0).getHeading());
        assertEquals(kHeadings.get(1), traj.getPoint(1).getHeading());
        assertEquals(kHeadings.get(2), traj.getPoint(2).getHeading());
        assertEquals(kHeadings.get(3), traj.getPoint(3).getHeading());
    }

    /**
     * Note that many of the results here are "wrong" because the waypoints aren't
     * correctly specified.
     */
    @Test
    void test() throws TimingException {
        List<Pose2dWithMotion> waypoints = Arrays.asList(
                new Pose2dWithMotion(
                        new Pose2d(new Translation2d(0.0, 0.0), GeometryUtil.fromDegrees(0)),
                        new MotionDirection(1, 0, 0.1), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(new Translation2d(24.0, 0.0), GeometryUtil.fromDegrees(30)),
                        new MotionDirection(1, 0, 0.1), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(new Translation2d(36.0, 0.0), GeometryUtil.fromDegrees(60)),
                        new MotionDirection(0, 1, 1e6), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(new Translation2d(36.0, 24.0), GeometryUtil.fromDegrees(60)),
                        new MotionDirection(1, 0, 0.1), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(new Translation2d(60.0, 24.0), GeometryUtil.fromDegrees(180)),
                        new MotionDirection(1, 0, 0.1), 0, 0));

        // Create the reference trajectory (straight line motion between waypoints).
        Path100 path = new Path100(waypoints);

        assertEquals(0.0, path.getMinDistance(), kDelta);
        // the total path length is a bit more than the straight-line path because each
        // path is a constant-twist arc.
        assertEquals(89.435, path.getMaxDistance(), kDelta);

        // initial sample is exactly at the start
        Pose2dWithMotion sample0 = path.sample(0.0);
        assertEquals(0, sample0.getPose().getX(), kDelta);
        assertEquals(0, sample0.getPose().getY(), kDelta);

        // course is +x
        assertEquals(0, sample0.getCourse().get().getDegrees());

        // heading is 0
        assertEquals(0, sample0.getHeading().getDegrees());

        // these are constant-twist paths, so they are little arcs.
        // halfway between 0 and 1, the path sags a little, and it's a little longer,
        // so this is not (12,0).
        Pose2dWithMotion sample12 = path.sample(12.0);
        assertEquals(11.862, sample12.getPose().getX(), kDelta);
        assertEquals(-1.58, sample12.getPose().getY(), kDelta);

        // course should be +x
        assertEquals(0, sample12.getCourse().get().getDegrees(), kDelta);

        // heading should be about 15 degrees, but since the path is a bit
        // longer than the straight line, we're not quite to the middle of it yet
        assertEquals(14.829, sample12.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion sample5 = path.sample(48);
        assertEquals(36, sample5.getPose().getX(), kDelta);
        assertEquals(11.585, sample5.getPose().getY(), kDelta);
        assertEquals(46.978, sample5.getCourse().get().getDegrees(), kDelta);
        assertEquals(60, sample5.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion sample6 = path.sample(60);
        assertEquals(36, sample6.getPose().getX(), kDelta);
        assertEquals(23.585, sample6.getPose().getY(), kDelta);
        assertEquals(1.006, sample6.getCourse().get().getDegrees(), kDelta);
        assertEquals(60, sample6.getHeading().getDegrees(), kDelta);

        // halfway between the last two points, the path sags a little,
        // and it's a little longer, so this is not (48,0)
        Pose2dWithMotion sample72 = path.sample(72.0);
        assertEquals(45.097, sample72.getPose().getX(), kDelta);
        assertEquals(17.379, sample72.getPose().getY(), kDelta);

        // course should be +x
        assertEquals(0, sample72.getCourse().get().getDegrees(), kDelta);

        // heading should be about 135 degrees, but because of the longer
        // paths, we're not quite to the center of the arc yet
        assertEquals(107.905, sample72.getHeading().getDegrees(), kDelta);

        Pose2dWithMotion sample8 = path.sample(84);
        assertEquals(56.440, sample8.getPose().getX(), kDelta);
        assertEquals(19.939, sample8.getPose().getY(), kDelta);
        assertEquals(0, sample8.getCourse().get().getDegrees(), kDelta);
        assertEquals(157.525, sample8.getHeading().getDegrees(), kDelta);

    }
}
