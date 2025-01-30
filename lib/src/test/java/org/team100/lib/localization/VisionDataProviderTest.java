package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Takt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

class VisionDataProviderTest implements Timeless {
    private static final double kDelta = 0.01;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testEstimateRobotPose() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        // these lists receive the updates
        final List<Pose2d> poseEstimate = new ArrayList<Pose2d>();
        final List<Double> timeEstimate = new ArrayList<Double>();

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                poseEstimate.add(p);
                timeEstimate.add(t);
            }

            @Override
            public SwerveModel get(double timestampSeconds) {
                return new SwerveModel(GeometryUtil.kRotationZero);
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(
                logger, layout, poseEstimator);

        // in red layout blip 7 is on the other side of the field

        // one meter range (Z forward)
        Blip24 blip = new Blip24(7, new Transform3d(new Translation3d(0, 0, 1), new Rotation3d()));

        // verify tag location
        Pose3d tagPose = layout.getTagPose(Alliance.Red, 7).get();
        assertEquals(3.658, tagPose.getX(), kDelta);
        assertEquals(4.026, tagPose.getY(), kDelta);
        assertEquals(0.308, tagPose.getZ(), kDelta);
        assertEquals(0, tagPose.getRotation().getX(), kDelta);
        assertEquals(0, tagPose.getRotation().getY(), kDelta);
        assertEquals(0, tagPose.getRotation().getZ(), kDelta);

        final String key = "foo";
        final Blip24[] blips = new Blip24[] {
                blip
        };

        vdp.estimateRobotPose(key, blips, Takt.get(), Alliance.Red);
        // do it twice to convince vdp it's a good estimate
        vdp.estimateRobotPose(key, blips, Takt.get(), Alliance.Red);
        assertEquals(1, poseEstimate.size());
        assertEquals(1, timeEstimate.size());

        Pose2d result = poseEstimate.get(0);
        assertEquals(2.657, result.getX(), kDelta); // target is one meter in front
        assertEquals(4.026, result.getY(), kDelta); // same y as target
        assertEquals(0, result.getRotation().getRadians(), kDelta); // facing along x
    }

    @Test
    void testEstimateRobotPose2() throws IOException {
        // robot is panned right 45, translation is ignored.
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        final List<Pose2d> poseEstimate = new ArrayList<Pose2d>();
        final List<Double> timeEstimate = new ArrayList<Double>();
        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                poseEstimate.add(p);
                timeEstimate.add(t);
            }

            @Override
            public SwerveModel get(double timestampSeconds) {
                return new SwerveModel(new Rotation2d(-Math.PI / 4));
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(
                logger, layout, poseEstimator);

        // camera sees the tag straight ahead in the center of the frame,
        // but rotated pi/4 to the left. this is ignored anyway.
        Blip24 blip = new Blip24(7, new Transform3d(
                new Translation3d(0, 0, Math.sqrt(2)),
                new Rotation3d(0, -Math.PI / 4, 0)));

        // verify tag 7 location
        Pose3d tagPose = layout.getTagPose(Alliance.Red, 7).get();
        assertEquals(3.658, tagPose.getX(), kDelta);
        assertEquals(4.026, tagPose.getY(), kDelta);
        assertEquals(0.308, tagPose.getZ(), kDelta);
        assertEquals(0, tagPose.getRotation().getX(), kDelta);
        assertEquals(0, tagPose.getRotation().getY(), kDelta);
        assertEquals(0, tagPose.getRotation().getZ(), kDelta);

        // default camera offset is no offset.
        final String cameraSerialNumber = "foo";
        final Blip24[] blips = new Blip24[] { blip };

        vdp.estimateRobotPose(cameraSerialNumber, blips, Takt.get() - 0.075, Alliance.Red);

        // two good estimates are required, so do another one.
        vdp.estimateRobotPose(cameraSerialNumber, blips, Takt.get() - 0.075, Alliance.Red);

        assertEquals(1, poseEstimate.size());
        assertEquals(1, timeEstimate.size());

        Pose2d result = poseEstimate.get(0);
        // robot is is one meter away from the target in x
        assertEquals(2.658, result.getX(), kDelta);
        // robot is one meter to the left (i.e. in y)
        assertEquals(5.026, result.getY(), kDelta);
        // facing diagonal, this is just what we provided.
        assertEquals(-Math.PI / 4, result.getRotation().getRadians(), kDelta);

        // the delay is just what we told it to use.
        double now = Takt.get();
        Double t = timeEstimate.get(0);
        double delay = now - t;
        assertEquals(0.075, delay, kDelta);
    }

    @Test
    void testRotationInterpolation() {
        // just to be sure of what it's doing
        Rotation2d a = Rotation2d.fromDegrees(10);
        Rotation2d b = Rotation2d.fromDegrees(340);
        Rotation2d c = a.interpolate(b, 0.5);
        assertEquals(-5, c.getDegrees(), kDelta);
    }

    @Test
    void testCase1() throws IOException {

        // the case from 2/14
        // robot 45 degrees to the right (negative), so 135 degrees
        // x = 2.2m, y = - 1.3 m from the center speaker tag
        // camera B
        // camera to tag 4: z=2.4, x=0, y=0 (approx)
        // camera to tag 3: z=2.8, x=0.1, y=0.1 (approx)
        // tag 4 in red is at about (0, 2.5)
        // tag 3 in red is at about (0, 3)

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                //
            }

            @Override
            public SwerveModel get(double timestampSeconds) {
                return new SwerveModel(new Rotation2d(3 * Math.PI / 4));
            }
        };
        VisionDataProvider24 vdp = new VisionDataProvider24(
                logger, layout, poseEstimator);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 2.4),
                new Rotation3d()));
        Blip24 tag3 = new Blip24(3, new Transform3d(
                new Translation3d(0.1, 0.1, 2.8),
                new Rotation3d()));

        final String cameraSerialNumber = "1000000013c9c96c";
        final Blip24[] tags = new Blip24[] { tag3, tag4 };

        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
    }

    @Test
    void testCase2() throws IOException {

        // 1m in front of tag 4
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), kDelta);
        assertEquals(1.914, tag4pose.getY(), kDelta);
        assertEquals(1.868, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), kDelta);
                assertEquals(1.914, p.getY(), kDelta);
            }

            @Override
            public SwerveModel get(double timestampSeconds) {
                return new SwerveModel(new Rotation2d(Math.PI));
            }
        };
        VisionDataProvider24 vdp = new VisionDataProvider24(
                logger, layout, poseEstimator);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));

        // test camera has zero offset
        final String cameraSerialNumber = "test";
        final Blip24[] tags = new Blip24[] { tag4 };

        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
    }

    @Test
    void testCase2WithOffset() throws IOException {
        // 1m in front of tag 4
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), kDelta);
        assertEquals(1.914, tag4pose.getY(), kDelta);
        assertEquals(1.868, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(10.272, p.getX(), kDelta);
                assertEquals(1.914, p.getY(), kDelta);
            }

            @Override
            public SwerveModel get(double timestampSeconds) {
                return new SwerveModel(new Rotation2d(Math.PI));
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(
                logger, layout, poseEstimator);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));

        // test2 camera is 1m in front, so robot is 1m further away.
        final String cameraSerialNumber = "test2";
        final Blip24[] tags = new Blip24[] { tag4 };

        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
    }

    @Test
    void testCase2WithTriangulation() throws IOException {

        // 1m in front of tag 4
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), kDelta);
        assertEquals(1.914, tag4pose.getY(), kDelta);
        assertEquals(1.868, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(0.96, p.getX(), kDelta);
                assertEquals(2.66, p.getY(), kDelta);
            }

            @Override
            public SwerveModel get(double timestampSeconds) {
                return new SwerveModel(new Rotation2d(Math.PI));
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(
                logger, layout, poseEstimator);

        Blip24 tag3 = new Blip24(3, new Transform3d(
                new Translation3d(0.561, 0, 1),
                new Rotation3d()));
        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));

        // test camera has zero offset
        final String cameraSerialNumber = "test";
        final Blip24[] tags = new Blip24[] { tag3, tag4 };

        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
    }

    @Test
    void testCase2tilt() throws IOException {

        // 1m in front of tag 4, tilted up 45
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), kDelta);
        assertEquals(1.914, tag4pose.getY(), kDelta);
        assertEquals(1.868, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), kDelta);
                assertEquals(1.914, p.getY(), kDelta);
            }

            @Override
            public SwerveModel get(double timestampSeconds) {
                return new SwerveModel(new Rotation2d(Math.PI));
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(
                logger, layout, poseEstimator);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));

        // test camera has zero offset
        final String cameraSerialNumber = "test1";
        final Blip24[] tags = new Blip24[] { tag4 };

        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
    }

    @Test
    void testCase3() throws IOException {

        // 1m in front of tag 4, 1m to the right
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), kDelta);
        assertEquals(1.914, tag4pose.getY(), kDelta);
        assertEquals(1.868, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), kDelta);
                assertEquals(2.914, p.getY(), kDelta);
            }

            @Override
            public SwerveModel get(double timestampSeconds) {
                return new SwerveModel(new Rotation2d(Math.PI));
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(
                logger, layout, poseEstimator);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(-1, 0, 1),
                new Rotation3d()));

        // test camera has zero offset
        final String cameraSerialNumber = "test";
        final Blip24[] tags = new Blip24[] { tag4 };

        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
    }

    @Test
    void testCase4() throws IOException {

        // 1m in front of tag 4, 1m to the right, rotated to the left
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), kDelta);
        assertEquals(1.914, tag4pose.getY(), kDelta);
        assertEquals(1.868, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), kDelta);
                assertEquals(2.914, p.getY(), kDelta);
            }

            @Override
            public SwerveModel get(double timestampSeconds) {
                return new SwerveModel(new Rotation2d(-3 * Math.PI / 4));
            }
        };

        VisionDataProvider24 vdp = new VisionDataProvider24(
                logger, layout, poseEstimator);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));

        // test camera has zero offset
        final String cameraSerialNumber = "test";
        final Blip24[] tags = new Blip24[] { tag4 };

        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
    }

    @Test
    void testCase5() throws IOException {

        // 1m in front of tag 4, 1m to the left, rotated to the right
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), kDelta);
        assertEquals(1.914, tag4pose.getY(), kDelta);
        assertEquals(1.868, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), kDelta);
                assertEquals(0.914, p.getY(), kDelta);
            }

            @Override
            public SwerveModel get(double timestampSeconds) {
                return new SwerveModel(new Rotation2d(3 * Math.PI / 4));
            }
        };
        VisionDataProvider24 vdp = new VisionDataProvider24(
                logger, layout, poseEstimator);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));

        // test camera has zero offset
        final String cameraSerialNumber = "test";
        final Blip24[] tags = new Blip24[] { tag4 };

        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
    }

    @Test
    void testCase6() throws IOException {

        // 1m in front of tag 4, 1m to the left, rotated to the right
        // looking up at a 45 degree angle
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), kDelta);
        assertEquals(1.914, tag4pose.getY(), kDelta);
        assertEquals(1.868, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), kDelta);
                assertEquals(0.914, p.getY(), kDelta);
            }

            @Override
            public SwerveModel get(double timestampSeconds) {
                return new SwerveModel(new Rotation2d(3 * Math.PI / 4));
            }
        };
        VisionDataProvider24 vdp = new VisionDataProvider24(
                logger, layout, poseEstimator);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 2),
                new Rotation3d()));

        // test1 camera is tilted up 45 degrees
        final String cameraSerialNumber = "test1";
        final Blip24[] tags = new Blip24[] { tag4 };

        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
    }

    @Test
    void testCase7() throws IOException {

        // 1m in front of tag 4, 1m to the left, rotated to the right
        // looking up at a 30 degree angle
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), kDelta);
        assertEquals(1.914, tag4pose.getY(), kDelta);
        assertEquals(1.868, tag4pose.getZ(), kDelta);

        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), kDelta);
                assertEquals(0.914, p.getY(), kDelta);
            }

            @Override
            public SwerveModel get(double timestampSeconds) {
                return new SwerveModel(new Rotation2d(3 * Math.PI / 4));
            }
        };
        VisionDataProvider24 vdp = new VisionDataProvider24(
                logger, layout, poseEstimator);

        // 30 degrees, long side is sqrt2, so hypotenuse is sqrt2/sqrt3/2
        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.633),
                new Rotation3d()));

        // test3 camera is tilted up 30 degrees
        final String cameraSerialNumber = "test3";
        final Blip24[] tags = new Blip24[] { tag4 };

        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
        vdp.estimateRobotPose(cameraSerialNumber, tags, Takt.get(), Alliance.Red);
    }
}