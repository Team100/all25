package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.geometry.PathPoint;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class CentripetalAccelerationConstraintTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final double CENTRIPETAL_SCALE = 1.0;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSimple() {
        assertEquals(8.166, SwerveKinodynamicsFactory.forTest(logger).getMaxCapsizeAccelM_S2(), DELTA);

        // 1 rad/m curve, 8 m/s^2 limit => 2.8 m/s
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                logger,
                SwerveKinodynamicsFactory.forTest(logger),
                CENTRIPETAL_SCALE);
        PathPoint p = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 1);
        // motionless, so 100% of the capsize accel is available
        assertEquals(-8.166, c.maxDecel(p, 0), DELTA);
        assertEquals(8.166, c.maxAccel(p, 0), DELTA);
        assertEquals(2.857, c.maxV(p), DELTA);
    }

    @Test
    void testSimpleMoving() {
        assertEquals(8.166, SwerveKinodynamicsFactory.forTest(logger).getMaxCapsizeAccelM_S2(), DELTA);

        // 1 rad/m curve, 8 m/s^2 limit => 2.8 m/s
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                logger,
                SwerveKinodynamicsFactory.forTest(logger),
                CENTRIPETAL_SCALE);
        PathPoint p = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 1);
        // moving, only some of the capsize accel is available
        assertEquals(-5.257, c.maxDecel(p, 2.5), DELTA);
        assertEquals(5.257, c.maxAccel(p, 2.5), DELTA);
        assertEquals(2.857, c.maxV(p), DELTA);
    }

    @Test
    void testSimpleOverspeed() {
        assertEquals(8.166, SwerveKinodynamicsFactory.forTest(logger).getMaxCapsizeAccelM_S2(), DELTA);

        // 1 rad/m curve, 8 m/s^2 limit => 2.8 m/s
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                logger,
                SwerveKinodynamicsFactory.forTest(logger),
                CENTRIPETAL_SCALE);
        PathPoint p = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 1);
        // above the velocity limit
        assertEquals(-1, c.maxDecel(p, 3), DELTA);
        assertEquals(0, c.maxAccel(p, 3), DELTA);
        assertEquals(2.857, c.maxV(p), DELTA);
    }

    @Test
    void testSimple2() {
        assertEquals(4.083, SwerveKinodynamicsFactory.forTest2(logger).getMaxCapsizeAccelM_S2(), DELTA);
        // 1 rad/m curve, 4 m/s^2 limit => 2 m/s
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                logger,
                SwerveKinodynamicsFactory.forTest2(logger),
                CENTRIPETAL_SCALE);
        PathPoint p = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 1);
        assertEquals(-4.083, c.maxDecel(p, 0), DELTA);
        assertEquals(4.083, c.maxAccel(p, 0), DELTA);
        assertEquals(2.021, c.maxV(p), DELTA);
    }

    @Test
    void testStraightLine() {
        assertEquals(4.083, SwerveKinodynamicsFactory.forTest2(logger).getMaxCapsizeAccelM_S2(), DELTA);
        // no curvature
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                logger,
                SwerveKinodynamicsFactory.forTest2(logger),
                CENTRIPETAL_SCALE);
        PathPoint p = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 0);
        assertEquals(-4.083, c.maxDecel(p, 0), DELTA);
        assertEquals(4.083, c.maxAccel(p, 0), DELTA);
        assertEquals(Double.POSITIVE_INFINITY, c.maxV(p), DELTA);
    }

}
