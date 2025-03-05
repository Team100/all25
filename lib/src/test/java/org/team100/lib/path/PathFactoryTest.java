package org.team100.lib.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.spline.HolonomicSpline;
import org.team100.lib.timing.ScheduleGenerator;
import org.team100.lib.timing.ScheduleGenerator.TimingException;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class PathFactoryTest {
    private static final boolean DEBUG = false;
    private static final double kDelta = 0.01;

    @Test
    void testForced() throws TimingException {
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d()),
                new Pose2d(1, 1, new Rotation2d()));
        Path100 path = PathFactory.withoutControlPoints(waypoints, 0.01, 0.01, 0.1);
        // sample the path so we can see it
        for (double t = 0; t <= path.getMaxDistance(); t += 0.1) {
            if (DEBUG)
                Util.printf("%.1f %5.2f %5.2f\n",
                        t,
                        path.sample(t).getPose().getX(),
                        path.sample(t).getPose().getY());
        }
        // schedule it so we can see it
        // no constraints
        TimingConstraintFactory f = new TimingConstraintFactory(SwerveKinodynamicsFactory.forTest());
        List<TimingConstraint> constraints = f.forTest();
        ScheduleGenerator scheduler = new ScheduleGenerator(constraints);
        Trajectory100 trajectory = scheduler.timeParameterizeTrajectory(
                path,
                0.0127,
                0.05,
                0.05);

        for (double t = 0; t < trajectory.duration(); t += 0.1) {
            if (DEBUG)
                Util.printf("%.1f %5.2f %5.2f\n",
                        t,
                        trajectory.sample(t).state().getPose().getX(),
                        trajectory.sample(t).state().getPose().getY());
        }

        assertEquals(13, path.length());
        Pose2dWithMotion p = path.getPoint(0);
        assertEquals(0, p.getPose().getX(), kDelta);
        assertEquals(0, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
        p = path.getPoint(1);
        assertEquals(0.3, p.getPose().getX(), kDelta);
        assertEquals(0, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
    }

    @Test
    void testBackingUp() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(0, 0), Rotation2d.kZero, new Rotation2d(Math.PI)),
                new HolonomicPose2d(new Translation2d(1, 0), Rotation2d.kZero, Rotation2d.kZero));
        Path100 path = PathFactory.pathFromWaypoints(
                waypoints,
                0.0127,
                0.0127,
                Math.toRadians(1.0));
        assertEquals(10, path.length());
    }

    /**
     * Stationary pure-rotation paths don't work.
     */
    @Test
    void testRotation() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(), new Rotation2d(1), new Rotation2d()));
        assertThrows(IllegalArgumentException.class,
                () -> PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1));
    }

    /** Preserves the tangent at the corner and so makes a little "S" */
    @Test
    void testCorner() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(0, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 1), new Rotation2d(), new Rotation2d(Math.PI / 2)));
        Path100 path = PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1);

        assertEquals(9, path.length());
        Pose2dWithMotion p = path.getPoint(0);
        assertEquals(0, p.getPose().getX(), kDelta);
        assertEquals(0, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
        p = path.getPoint(1);
        assertEquals(1, p.getPose().getX(), kDelta);
        assertEquals(0, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
    }

    /**
     * Stationary paths don't work.
     */
    @Test
    void testStationary() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()));
        assertThrows(IllegalArgumentException.class,
                () -> PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1));
    }

    @Test
    void testLinear() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d()));
        Path100 path = PathFactory.pathFromWaypoints(
                waypoints, 0.01, 0.01, 0.1);
        assertEquals(2, path.length());
        Pose2dWithMotion p = path.getPoint(0);
        assertEquals(0, p.getPose().getX(), kDelta);
        assertEquals(0, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
        p = path.getPoint(1);
        assertEquals(1, p.getPose().getX(), kDelta);
        assertEquals(0, p.getPose().getRotation().getRadians(), kDelta);
        assertEquals(0, p.getHeadingRate(), kDelta);
    }

    @Test
    void testActualCorner() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(0, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d(Math.PI / 2)),
                new HolonomicPose2d(new Translation2d(1, 1), new Rotation2d(), new Rotation2d(Math.PI / 2)));
        assertThrows(IllegalArgumentException.class,
                () -> PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1));
    }

    @Test
    void testComposite() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(0, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(1), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(2, 0), new Rotation2d(1), new Rotation2d()));
        assertThrows(IllegalArgumentException.class,
                () -> PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1));
    }

    @Test
    void test() {
        HolonomicPose2d p1 = new HolonomicPose2d(new Translation2d(0, 0), Rotation2d.kZero, Rotation2d.kZero);
        HolonomicPose2d p2 = new HolonomicPose2d(new Translation2d(15, 10), Rotation2d.kZero, new Rotation2d(1, 5));
        HolonomicSpline s = new HolonomicSpline(p1, p2);

        List<Pose2dWithMotion> samples = PathFactory.parameterizeSpline(s, 0.05, 0.05, 0.1, 0.0, 1.0);

        double arclength = 0;
        Pose2dWithMotion cur_pose = samples.get(0);
        for (Pose2dWithMotion sample : samples) {
            Twist2d twist = GeometryUtil.slog(
                    GeometryUtil.transformBy(
                            GeometryUtil.inverse(cur_pose.getPose()), sample.getPose()));
            arclength += Math.hypot(twist.dx, twist.dy);
            cur_pose = sample;
        }

        assertEquals(15.0, cur_pose.getTranslation().getX(), 0.001);
        assertEquals(10.0, cur_pose.getTranslation().getY(), 0.001);
        assertEquals(78.690, cur_pose.getCourse().get().getDegrees(), 0.001);
        assertEquals(20.416, arclength, 0.001);
    }

    @Test
    void testDx() {
        // HolonomicSpline s0 = new HolonomicSpline(
        // new HolonomicPose2d(new Translation2d(0, 0), Rotation2d.kZero,
        // Rotation2d.kZero),
        // new HolonomicPose2d(new Translation2d(1, 0), Rotation2d.kZero,
        // Rotation2d.kZero),
        // 1.0);
        HolonomicSpline s0 = new HolonomicSpline(
                new HolonomicPose2d(new Translation2d(0, -1), Rotation2d.kZero, Rotation2d.kZero),
                new HolonomicPose2d(new Translation2d(1, 0), Rotation2d.kZero, Rotation2d.kCCW_90deg),
                1.0);
        List<HolonomicSpline> splines = List.of(s0);
        List<Pose2dWithMotion> motion = PathFactory.parameterizeSplines(splines, 0.001, 0.001, 0.001);
        for (Pose2dWithMotion p : motion) {
            Util.printf("%5.3f %5.3f\n", p.getTranslation().getX(), p.getTranslation().getY());
        }
    }

}
