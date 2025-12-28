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

public class ConstantConstraintTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testVelocity() {
        ConstantConstraint c = new ConstantConstraint(logger, 2, 3);
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 0);
        assertEquals(2, c.maxV(state), DELTA);
    }

    @Test
    void testAccel() {
        ConstantConstraint c = new ConstantConstraint(logger, 2, 3);
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 0);
        assertEquals(-3, c.maxDecel(state, 1), DELTA);
        assertEquals(3, c.maxAccel(state, 1), DELTA);

    }

}
