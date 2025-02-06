package org.team100.lib.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.geometry.Pose2d;

class CentripetalAccelerationConstraintTest {
    private static final double kDelta = 0.001;
    private static final double kCentripetalScale = 1.0;

    @Test
    void testSimple() {
        assertEquals(8.166, SwerveKinodynamicsFactory.forTest().getMaxCapsizeAccelM_S2(), kDelta);

        // 1 rad/m curve, 8 m/s^2 limit => 2.8 m/s
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                SwerveKinodynamicsFactory.forTest(),
                kCentripetalScale);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new MotionDirection(0, 0, 0), 1, 0);
        // motionless, so 100% of the capsize accel is available
        assertEquals(-8.166, c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(8.166, c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
        assertEquals(2.857, c.getMaxVelocity(p).getValue(), kDelta);
    }

    @Test
    void testSimpleMoving() {
        assertEquals(8.166, SwerveKinodynamicsFactory.forTest().getMaxCapsizeAccelM_S2(), kDelta);

        // 1 rad/m curve, 8 m/s^2 limit => 2.8 m/s
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                SwerveKinodynamicsFactory.forTest(),
                kCentripetalScale);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new MotionDirection(1, 0, 0), 1, 0);
        // moving, only some of the capsize accel is available
        assertEquals(-5.257, c.getMinMaxAcceleration(p, 2.5).getMinAccel(), kDelta);
        assertEquals(5.257, c.getMinMaxAcceleration(p, 2.5).getMaxAccel(), kDelta);
        assertEquals(2.857, c.getMaxVelocity(p).getValue(), kDelta);
    }

    @Test
    void testSimpleOverspeed() {
        assertEquals(8.166, SwerveKinodynamicsFactory.forTest().getMaxCapsizeAccelM_S2(), kDelta);

        // 1 rad/m curve, 8 m/s^2 limit => 2.8 m/s
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                SwerveKinodynamicsFactory.forTest(),
                kCentripetalScale);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new MotionDirection(1, 0, 0), 1, 0);
        // above the velocity limit
        assertEquals(-1, c.getMinMaxAcceleration(p, 3).getMinAccel(), kDelta);
        assertEquals(0, c.getMinMaxAcceleration(p, 3).getMaxAccel(), kDelta);
        assertEquals(2.857, c.getMaxVelocity(p).getValue(), kDelta);
    }

    @Test
    void testSimple2() {
        assertEquals(4.083, SwerveKinodynamicsFactory.forTest2().getMaxCapsizeAccelM_S2(), kDelta);
        // 1 rad/m curve, 4 m/s^2 limit => 2 m/s
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                SwerveKinodynamicsFactory.forTest2(),
                kCentripetalScale);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new MotionDirection(0, 0, 0), 1, 0);
        assertEquals(-4.083, c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(4.083, c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
        assertEquals(2.021, c.getMaxVelocity(p).getValue(), kDelta);
    }

    @Test
    void testStraightLine() {
        assertEquals(4.083, SwerveKinodynamicsFactory.forTest2().getMaxCapsizeAccelM_S2(), kDelta);
        // no curvature
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                SwerveKinodynamicsFactory.forTest2(),
                kCentripetalScale);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new MotionDirection(1, 0, 0), 0, 0);
        assertEquals(-4.083, c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(4.083, c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMaxVelocity(p).getValue(), kDelta);
    }

}
