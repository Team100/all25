package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.geometry.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TorqueConstraintTest {
    private static final double DELTA = 0.001;

    @Test
    void testRadial() {
        TorqueConstraint jc = new TorqueConstraint(6);
        // moving +x at (1,0,0)
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(1, 0, new Rotation2d(0)), 0, 1.2),
                0, 0);
        // no tangential motion => no limit
        assertEquals(Double.NEGATIVE_INFINITY, jc.maxDecel(state, 0), DELTA);
        assertEquals(Double.POSITIVE_INFINITY, jc.maxAccel(state, 0), DELTA);
    }

    @Test
    void testTangential() {
        TorqueConstraint jc = new TorqueConstraint(6);
        // at (1,0,0), moving (0,1,0)
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(1, 0, new Rotation2d(0)), Math.PI / 2, 1.2),
                0, 0);
        // tangential motion at 1 m
        assertEquals(-1, jc.maxDecel(state, 0), DELTA);
        assertEquals(1, jc.maxAccel(state, 0), DELTA);
    }

    @Test
    void testInclined() {
        TorqueConstraint jc = new TorqueConstraint(6);
        // moving +x+y at (1,0,0)
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(1, 0, new Rotation2d(0)), Math.PI / 4, 1.2),
                0, 0);
        // motion at 45 deg => higher limit
        assertEquals(-1.414, jc.maxDecel(state, 0), DELTA);
        assertEquals(1.414, jc.maxAccel(state, 0), DELTA);
    }

    @Test
    void testFar() {
        TorqueConstraint jc = new TorqueConstraint(6);
        // moving +y at (2,0,0)
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(2, 0, new Rotation2d(0)), Math.PI / 2, 1.2),
                0, 0);
        // more r => lower limit
        assertEquals(-0.5, jc.maxDecel(state, 0), DELTA);
        assertEquals(0.5, jc.maxAccel(state, 0), DELTA);
    }

    @Test
    void testFar2() {
        TorqueConstraint jc = new TorqueConstraint(6);
        // moving +y at (3,0,0)
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(3, 0, new Rotation2d(0)), Math.PI / 2, 1.2),
                0, 0);
        // more r => lower limit
        assertEquals(-0.333, jc.maxDecel(state, 0), DELTA);
        assertEquals(0.333, jc.maxAccel(state, 0), DELTA);
    }

    @Test
    void testRealistic() {
        TorqueConstraint jc = new TorqueConstraint(30);
        // moving +y at (1,0,0)
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(1, 0, new Rotation2d(0)), Math.PI / 2, 1.2),
                0, 0);
        // should match the constant constraint at around 1 m
        assertEquals(-5, jc.maxDecel(state, 0), DELTA);
        assertEquals(5, jc.maxAccel(state, 0), DELTA);
    }
}
