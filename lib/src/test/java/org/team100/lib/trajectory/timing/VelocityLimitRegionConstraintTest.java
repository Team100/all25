package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.geometry.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class VelocityLimitRegionConstraintTest {
    private static final double DELTA = 0.001;

    @Test
    void testOutside() {
        // towards +x, 1 rad/m, 1 rad/s limit => 1 m/s
        VelocityLimitRegionConstraint c = new VelocityLimitRegionConstraint(
                new Translation2d(), new Translation2d(1, 1), 1);
        PathPoint p = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(-1, -1, new Rotation2d(0)), 0, 1.2),
                0, // spatial, so rad/m
                0);
        assertEquals(Double.NEGATIVE_INFINITY, c.maxDecel(p, 0), DELTA);
        assertEquals(Double.POSITIVE_INFINITY, c.maxAccel(p, 0), DELTA);
        assertEquals(Double.POSITIVE_INFINITY, c.maxV(p), DELTA);
    }

    @Test
    void testInside() {
        // towards +x, 1 rad/m, 1 rad/s limit => 1 m/s
        VelocityLimitRegionConstraint c = new VelocityLimitRegionConstraint(
                new Translation2d(), new Translation2d(1, 1), 1);
        PathPoint p = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0.5, 0.5, new Rotation2d(0)), 0, 1.2),
                0, // spatial, so rad/m
                0);
        assertEquals(Double.NEGATIVE_INFINITY, c.maxDecel(p, 0), DELTA);
        assertEquals(Double.POSITIVE_INFINITY, c.maxAccel(p, 0), DELTA);
        assertEquals(1, c.maxV(p), DELTA);
    }

}
