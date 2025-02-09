package org.team100.lib.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.timing.TimingConstraint.MinMaxAcceleration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveDriveDynamicsConstraintTest {
    private static final double kDelta = 0.001;

    // the free speed of a module, which is also the free speed
    // of the robot going in a straight line without rotating.
    private static final double maxV = 4;

    @Test
    void testVelocity() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        SwerveDriveDynamicsConstraint c = new SwerveDriveDynamicsConstraint(l);

        // motionless
        double m = c.getMaxVelocity(Pose2dWithMotion.kIdentity).getValue();
        assertEquals(5, m, kDelta);

        // moving in +x, no curvature, no rotation
        m = c.getMaxVelocity(new Pose2dWithMotion(
                new Pose2d(),
                new MotionDirection(1, 0, 0),
                0, 0)).getValue();
        // max allowed velocity is full speed
        assertEquals(5, m, kDelta);

        // moving in +x, 5 rad/meter
        m = c.getMaxVelocity(new Pose2dWithMotion(
                new Pose2d(),
                new MotionDirection(1, 0, 5),
                0, 0)).getValue();
        // at 5 rad/m with 0.5m sides the fastest you can go is 1.55 m/s.
        assertEquals(1.887, m, kDelta);
    }

    @Test
    void testAccel() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        SwerveDriveDynamicsConstraint c = new SwerveDriveDynamicsConstraint(l);
        // this is constant
        MinMaxAcceleration m = c.getMinMaxAcceleration(Pose2dWithMotion.kIdentity, 0);
        assertEquals(-20, m.getMinAccel(), kDelta);
        assertEquals(10, m.getMaxAccel(), kDelta);
    }
}
