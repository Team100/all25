package org.team100.lib.trajectory.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.PathPoint;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.path.spline.HolonomicSpline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class Path100Test {

    private static final List<Rotation2d> HEADINGS = Arrays.asList(
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(30),
            GeometryUtil.fromDegrees(60),
            GeometryUtil.fromDegrees(90));

    private static final List<PathPoint> WAYPOINTS = Arrays.asList(
            new PathPoint(
                    WaypointSE2.irrotational(
                            new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                    0, 0),
            new PathPoint(
                    WaypointSE2.irrotational(
                            new Pose2d(24, 0, new Rotation2d(Math.toRadians(30))), 0, 1.2),
                    0, 0),
            new PathPoint(
                    WaypointSE2.irrotational(
                            new Pose2d(36, 12, new Rotation2d(Math.toRadians(60))), 0, 1.2),
                    0, 0),
            new PathPoint(
                    WaypointSE2.irrotational(
                            new Pose2d(60, 12, new Rotation2d(Math.toRadians(90))), 0, 1.2),
                    0, 0));

    @Test
    void testEmpty() {
        List<HolonomicSpline> splines = new ArrayList<>();
        PathFactory pathFactory = new PathFactory(0.1, 0.1, 0.1, 0.1);
        Path100 path = pathFactory.fromSplines(splines);
        assertEquals(0, path.length(), 0.001);
    }

    @Test
    void testConstruction() {
        Path100 traj = new Path100(WAYPOINTS);
        assertFalse(traj.isEmpty());
        assertEquals(4, traj.length());
    }

    @Test
    void testStateAccessors() {
        Path100 traj = new Path100(WAYPOINTS);

        assertEquals(WAYPOINTS.get(0), traj.getPoint(0));
        assertEquals(WAYPOINTS.get(1), traj.getPoint(1));
        assertEquals(WAYPOINTS.get(2), traj.getPoint(2));
        assertEquals(WAYPOINTS.get(3), traj.getPoint(3));

        assertEquals(HEADINGS.get(0), traj.getPoint(0).waypoint().pose().getRotation());
        assertEquals(HEADINGS.get(1), traj.getPoint(1).waypoint().pose().getRotation());
        assertEquals(HEADINGS.get(2), traj.getPoint(2).waypoint().pose().getRotation());
        assertEquals(HEADINGS.get(3), traj.getPoint(3).waypoint().pose().getRotation());
    }

}
