package org.team100.lib.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.timing.ScheduleGenerator.TimingException;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathPlannerTest {
    private static final boolean DEBUG = false;
    private static final double kDelta = 0.01;

    @Test
    void testForced() throws TimingException {
        List<Translation2d> waypoints = List.of(
                new Translation2d(0, 0),
                new Translation2d(1, 0),
                new Translation2d(1, 1));
        List<Rotation2d> headings = List.of(
                new Rotation2d(),
                new Rotation2d(),
                new Rotation2d());
        Path100 path = PathPlanner.withoutControlPoints(waypoints, headings, 0.01, 0.01, 0.1);
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
        TrajectoryPlanner tPlan = new TrajectoryPlanner(constraints);
        Trajectory100 trajectory = tPlan.generateTrajectory(path, 0.05, 0.05);

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
        Path100 path = PathPlanner.pathFromWaypoints(
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
                () -> PathPlanner.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1));
    }

    /** Preserves the tangent at the corner and so makes a little "S" */
    @Test
    void testCorner() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(0, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 1), new Rotation2d(), new Rotation2d(Math.PI / 2)));
        Path100 path = PathPlanner.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1);

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
                () -> PathPlanner.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1));
    }

    @Test
    void testLinear() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d()));
        Path100 path = PathPlanner.pathFromWaypoints(
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
                () -> PathPlanner.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1));
    }

    @Test
    void testComposite() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(0, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(1), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(2, 0), new Rotation2d(1), new Rotation2d()));
        assertThrows(IllegalArgumentException.class,
                () -> PathPlanner.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1));
    }
}
