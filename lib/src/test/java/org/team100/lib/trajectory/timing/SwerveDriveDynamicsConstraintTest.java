package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.geometry.PathPoint;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class SwerveDriveDynamicsConstraintTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testVelocity() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest(logger);
        SwerveDriveDynamicsConstraint c = new SwerveDriveDynamicsConstraint(logger, kinodynamics, 1, 1);

        // motionless
        double m = c.maxV(new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 0));
        assertEquals(5, m, DELTA);

        // moving in +x, no curvature, no rotation
        m = c.maxV(new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                0, 0));
        // max allowed velocity is full speed
        assertEquals(5, m, DELTA);

        // moving in +x, 5 rad/meter
        m = c.maxV(new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                5, 0));
        // at 5 rad/m with 0.5m sides the fastest you can go is 1.55 m/s.
        assertEquals(1.925, m, DELTA);

        // max wheel speed 5 m/s
        // wheelsbase/track 0.5 m
        // so radius to center is 0.25 * sqrt(2) = 0.356
        // traveling 1 m/s, there are 4 m/s available for the fastest wheel
        // which means 11.314 rad/s, and also 11.314 rad/m since we're going 1 m/s.
        PathPoint state = new PathPoint(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                11.313708, 0);
        m = c.maxV(state);
        // verify corner velocity is full scale
        assertEquals(5, c.maxV());
        // this should be feasible; note it's not exactly 1 due to discretization
        assertEquals(1.036, m, DELTA);

    }

    @Test
    void testAccel() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest(logger);
        SwerveDriveDynamicsConstraint c = new SwerveDriveDynamicsConstraint(logger, kinodynamics, 1, 1);
        // this is constant
        Pose2d p = new Pose2d(0, 0, new Rotation2d(0));
        PathPoint p2 = new PathPoint(
                WaypointSE2.irrotational(p, 0, 1.2), 0, 0);
        assertEquals(-20, c.maxDecel(p2, 0), DELTA);
        assertEquals(10, c.maxAccel(p2, 0), DELTA);
    }
}
