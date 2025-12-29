package org.team100.lib.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class GeometryUtilTest {
    private static final double DELTA = 0.001;

    @Test
    void testProject() {
        {
            ChassisSpeeds prev = new ChassisSpeeds(0.5, 0.5, 0);
            ChassisSpeeds target = new ChassisSpeeds(1, 0, 0);
            ChassisSpeeds result = GeometryUtil.project(prev, target);
            assertEquals(0.5, result.vxMetersPerSecond, 1e-12);
            assertEquals(0, result.vyMetersPerSecond, 1e-12);
            assertEquals(0, result.omegaRadiansPerSecond, 1e-12);
            assertEquals(0, GeometryUtil.getCourse(result).get().getRadians(), 1e-12);
        }
        {
            ChassisSpeeds prev = new ChassisSpeeds(0.5, 0.5, 1);
            ChassisSpeeds target = new ChassisSpeeds(1, 0, 0);
            ChassisSpeeds result = GeometryUtil.project(prev, target);
            assertEquals(0.5, result.vxMetersPerSecond, 1e-12);
            assertEquals(0, result.vyMetersPerSecond, 1e-12);
            assertEquals(1, result.omegaRadiansPerSecond, 1e-12);
            assertEquals(0, GeometryUtil.getCourse(result).get().getRadians(), 1e-12);
        }
        {
            ChassisSpeeds prev = new ChassisSpeeds(0.5, 0.5, 1);
            ChassisSpeeds target = new ChassisSpeeds(2, 0, 1);
            ChassisSpeeds result = GeometryUtil.project(prev, target);
            assertEquals(0.5, result.vxMetersPerSecond, 1e-12);
            assertEquals(0, result.vyMetersPerSecond, 1e-12);
            assertEquals(1, result.omegaRadiansPerSecond, 1e-12);
            assertEquals(0, GeometryUtil.getCourse(result).get().getRadians(), 1e-12);
        }
        {
            ChassisSpeeds prev = new ChassisSpeeds(1, 0, 0);
            ChassisSpeeds target = new ChassisSpeeds(1, 0, 0);
            ChassisSpeeds result = GeometryUtil.project(prev, target);
            assertEquals(1, result.vxMetersPerSecond, 1e-12);
            assertEquals(0, result.vyMetersPerSecond, 1e-12);
            assertEquals(0, result.omegaRadiansPerSecond, 1e-12);
            assertEquals(0, GeometryUtil.getCourse(result).get().getRadians(), 1e-12);
        }
        {
            // project onto itself, no-op.
            ChassisSpeeds prev = new ChassisSpeeds(1.99996732, -0.01143327, 3.50000000);
            ChassisSpeeds target = new ChassisSpeeds(1.99996732, -0.01143327, 3.50000000);
            assertEquals(-0.005716666136460627, GeometryUtil.getCourse(target).get().getRadians(), 1e-12);
            ChassisSpeeds result = GeometryUtil.project(prev, target);
            assertEquals(1.99996732, result.vxMetersPerSecond, 1e-12);
            assertEquals(-0.01143327, result.vyMetersPerSecond, 1e-12);
            assertEquals(3.5, result.omegaRadiansPerSecond, 1e-12);
            assertEquals(-0.005716666136460627, GeometryUtil.getCourse(result).get().getRadians(), 1e-12);
        }
    }

    @Test
    void testSlog() {
        Twist2d twist = GeometryUtil.slog(
                new Pose2d(1, 0, Rotation2d.kZero));
        assertEquals(1, twist.dx, DELTA);
        assertEquals(0, twist.dy, DELTA);
        assertEquals(0, twist.dtheta, DELTA);

        // this twist represents an arc that ends up at the endpoint.
        twist = GeometryUtil.slog(
                new Pose2d(1, 0, Rotation2d.kCCW_Pi_2));
        assertEquals(0.785, twist.dx, DELTA);
        assertEquals(-0.785, twist.dy, DELTA);
        assertEquals(1.571, twist.dtheta, DELTA);
    }

    @Test
    void testCollinear() {
        assertTrue(GeometryUtil.isColinear(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d())));
        assertFalse(GeometryUtil.isColinear(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d(Math.PI))));
        assertTrue(GeometryUtil.isColinear(
                new Pose2d(0, 0, new Rotation2d(Math.PI / 2)),
                new Pose2d(0, 1, new Rotation2d(Math.PI / 2))));
        assertFalse(GeometryUtil.isColinear(
                new Pose2d(),
                new Pose2d(1, 1, new Rotation2d())));
    }

    @Test
    void testParallel() {
        assertTrue(GeometryUtil.isParallel(new Rotation2d(), new Rotation2d()));
        assertTrue(GeometryUtil.isParallel(new Rotation2d(), new Rotation2d(Math.PI)));
    }

    @Test
    void testZforwardToXforward1() {

        // camera coordinates are x-right, y-down, z-forward
        // zero rotation.
        Rotation3d zforward = new Rotation3d();
        Quaternion q = zforward.getQuaternion();
        assertEquals(0, q.getX(), DELTA);
        assertEquals(0, q.getY(), DELTA);
        assertEquals(0, q.getZ(), DELTA);
        assertEquals(1, q.getW(), DELTA);

        // robot coordinates are x-forward, y-left, z-up
        // in this frame the rotation is still zero.
        Rotation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(0, xforward.getX(), DELTA);
        assertEquals(0, xforward.getY(), DELTA);
        assertEquals(0, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward2() {

        // camera coordinates are x-right, y-down, z-forward
        // 45 degree rotation around z.
        Rotation3d zforward = new Rotation3d(0, 0, Math.PI / 4);
        Quaternion q = zforward.getQuaternion();
        assertEquals(0, q.getX(), DELTA);
        assertEquals(0, q.getY(), DELTA);
        assertEquals(0.383, q.getZ(), DELTA);
        assertEquals(0.924, q.getW(), DELTA);

        // robot coordinates are x-forward, y-left, z-up
        // in this frame the rotation is around x.
        Rotation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(Math.PI / 4, xforward.getX(), DELTA);
        assertEquals(0, xforward.getY(), DELTA);
        assertEquals(0, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward3() {

        // camera coordinates are x-right, y-down, z-forward
        // 45 degree rotation around x. (tilt up)
        Rotation3d zforward = new Rotation3d(Math.PI / 4, 0, 0);
        Quaternion q = zforward.getQuaternion();
        assertEquals(0.383, q.getX(), DELTA);
        assertEquals(0, q.getY(), DELTA);
        assertEquals(0, q.getZ(), DELTA);
        assertEquals(0.924, q.getW(), DELTA);

        // robot coordinates are x-forward, y-left, z-up
        // in this frame the rotation is around y, negative.
        Rotation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(0, xforward.getX(), DELTA);
        assertEquals(-Math.PI / 4, xforward.getY(), DELTA);
        assertEquals(0, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward4() {
        // camera coordinates are x-right, y-down, z-forward
        // 45 degree rotation around y. (pan right)
        Rotation3d zforward = new Rotation3d(0, Math.PI / 4, 0);
        Quaternion q = zforward.getQuaternion();
        assertEquals(0, q.getX(), DELTA);
        assertEquals(0.383, q.getY(), DELTA);
        assertEquals(0, q.getZ(), DELTA);
        assertEquals(0.924, q.getW(), DELTA);

        // robot coordinates are x-forward, y-left, z-up
        // in this frame the rotation is around y, negative.
        Rotation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(0, xforward.getX(), DELTA);
        assertEquals(0, xforward.getY(), DELTA);
        assertEquals(-Math.PI / 4, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward5() {
        Translation3d zforward = new Translation3d();
        // still zero
        Translation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(0, xforward.getX(), DELTA);
        assertEquals(0, xforward.getY(), DELTA);
        assertEquals(0, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward6() {
        Translation3d zforward = new Translation3d(0, 0, 1);
        // still zero
        Translation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(1, xforward.getX(), DELTA);
        assertEquals(0, xforward.getY(), DELTA);
        assertEquals(0, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward7() {
        Translation3d zforward = new Translation3d(1, 0, 0);
        // still zero
        Translation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(0, xforward.getX(), DELTA);
        assertEquals(-1, xforward.getY(), DELTA);
        assertEquals(0, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward8() {
        Translation3d zforward = new Translation3d(0, 1, 0);
        // still zero
        Translation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(0, xforward.getX(), DELTA);
        assertEquals(0, xforward.getY(), DELTA);
        assertEquals(-1, xforward.getZ(), DELTA);
    }
}
