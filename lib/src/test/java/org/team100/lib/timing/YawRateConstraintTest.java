package org.team100.lib.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.geometry.Pose2d;

class YawRateConstraintTest {
    private static final double kDelta = 0.001;
    // for testing, use the aboslute maximum. This shouldn't be used in a real
    // robot.
    private static final double kYawRateScale = 1.0;

    @Test
    void testSpin() {
        // one radian/m in place i.e. no constraint
        YawRateConstraint c = new YawRateConstraint(SwerveKinodynamicsFactory.forTest(),
                kYawRateScale);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new MotionDirection(0, 0, 1), 0, 0);
        // the limiter is in the scheduler which only thinks about the along-the-path
        // dimension.
        // TODO: do a real holonomic scheduler
        assertEquals(Double.NEGATIVE_INFINITY,
                c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY,
                c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
        assertEquals(Double.MAX_VALUE, c.getMaxVelocity(p).getValue(), kDelta);
    }

    @Test
    void testNormal() {
        // towards +x, 1 rad/m, 1 m/s wheel -> 1 rad/s limit => 2.8 m/s (which violates
        // the linear constraint but it's ok)
        YawRateConstraint c = new YawRateConstraint(SwerveKinodynamicsFactory.forTest(),
                kYawRateScale);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(),
                new MotionDirection(1, 0, 1), // spatial, so rad/m
                0, 0);
        assertEquals(2.828, c.getMaxVelocity(p).getValue(), kDelta);
    }

    @Test
    void testVelocity2() {
        // towards +x, 1 rad/m, 2 rad/s limit => 2 m/s
        YawRateConstraint c = new YawRateConstraint(SwerveKinodynamicsFactory.forTest2(),
                kYawRateScale);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(),
                new MotionDirection(1, 0, 1), // spatial, so rad/m
                0, 0);
        assertEquals(5.656, c.getMaxVelocity(p).getValue(), kDelta);
    }

    @Test
    void testAccel() {
        // we should impose an accel limit, now that the trajectory builder doesn't
        // force omega to zero at the start.
        YawRateConstraint c = new YawRateConstraint(SwerveKinodynamicsFactory.forTest(),
                kYawRateScale);
        // driving and spinning
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(),
                new MotionDirection(1, 0, 1),
                0, 0);
        // there is an accel limit.
        assertEquals(-8.485,
                c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(8.485,
                c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
    }

    @Test
    void testAccel2() {
        // towards +x, 1 rad/m, 2 rad/s limit => 2 m/s
        double scale = 0.1;
        YawRateConstraint c = new YawRateConstraint(SwerveKinodynamicsFactory.forRealisticTest(),
                scale);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(),
                new MotionDirection(1, 0, 1), // spatial, so rad/m
                0, 0);
        // this number is still quite high even with a low scale.
        assertEquals(-16.971,
                c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(16.971,
                c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
    }
}
