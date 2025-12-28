package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.geometry.PathPoint;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DiamondConstraintTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSquare() {
        // here the two speeds are the same
        DiamondConstraint c = new DiamondConstraint(logger, 1, 1, 4);
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 0);
        // moving purely in x, get the x number
        assertEquals(1, c.maxV(state), DELTA);
        state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), Math.PI / 2, 1.2),
                0, 0);
        // moving purely in y, get the y number
        assertEquals(1, c.maxV(state), DELTA);
        state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), Math.PI / 4, 1.2),
                0, 0);
        // moving diagonally, get less.
        assertEquals(0.707, c.maxV(state), DELTA);
    }

    @Test
    void testVelocity() {
        DiamondConstraint c = new DiamondConstraint(logger, 2, 3, 4);
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 0);
        // moving purely in x, get the x number
        assertEquals(2, c.maxV(state), DELTA);
        state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), Math.PI / 2, 1.2),
                0, 0);
        // moving purely in y, get the y number
        assertEquals(3, c.maxV(state), DELTA);
        state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), Math.PI / 4, 1.2),
                0, 0);
        // moving diagonally, get less.
        assertEquals(1.697, c.maxV(state), DELTA);

    }

}
