package org.team100.lib.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PathPlannerTest {
    private static final double kDelta = 0.01;

    @Test
    void testBackingUp() {
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, new Rotation2d(Math.PI)),
                new Pose2d(1, 0, GeometryUtil.kRotationZero));
        List<Rotation2d> headings = List.of(
                GeometryUtil.kRotationZero,
                GeometryUtil.kRotationZero);
        Path100 path = PathPlanner.pathFromWaypointsAndHeadings(
                waypoints,
                headings,
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
        List<Pose2d> waypoints = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d(1));
        assertThrows(IllegalArgumentException.class,
                () -> PathPlanner.pathFromWaypointsAndHeadings(
                        waypoints, headings, 0.01, 0.01, 0.1));
    }

    /** Preserves the tangent at the corner and so makes a little "S" */
    @Test
    void testCorner() {
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d()),
                new Pose2d(1, 1, new Rotation2d(Math.PI / 2)));
        List<Rotation2d> headings = List.of(
                new Rotation2d(),
                new Rotation2d(),
                new Rotation2d());
        Path100 path = PathPlanner.pathFromWaypointsAndHeadings(
                waypoints, headings, 0.01, 0.01, 0.1);

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
        List<Pose2d> waypoints = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d());
        assertThrows(IllegalArgumentException.class,
                () -> PathPlanner.pathFromWaypointsAndHeadings(
                        waypoints, headings, 0.01, 0.01, 0.1));

    }

    @Test
    void testLinear() {
        List<Pose2d> waypoints = List.of(new Pose2d(), new Pose2d(1, 0, new Rotation2d()));
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d());
        Path100 path = PathPlanner.pathFromWaypointsAndHeadings(
                waypoints, headings, 0.01, 0.01, 0.1);

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

    /**
     * Paths with corners don't work.
     */
    @Test
    void testActualCorner() {
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d(Math.PI / 2)),
                new Pose2d(1, 1, new Rotation2d(Math.PI / 2)));
        List<Rotation2d> headings = List.of(
                new Rotation2d(),
                new Rotation2d(),
                new Rotation2d(),
                new Rotation2d());
        assertThrows(IllegalArgumentException.class,
                () -> PathPlanner.pathFromWaypointsAndHeadings(
                        waypoints, headings, 0.01, 0.01, 0.1));
    }

    /**
     * Partially-stationary paths don't work.
     */
    @Test
    void testComposite() {
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d()),
                new Pose2d(2, 0, new Rotation2d()));
        List<Rotation2d> headings = List.of(
                new Rotation2d(),
                new Rotation2d(),
                new Rotation2d(1),
                new Rotation2d(1));
        assertThrows(IllegalArgumentException.class,
                () -> PathPlanner.pathFromWaypointsAndHeadings(
                        waypoints, headings, 0.01, 0.01, 0.1));
    }
}
