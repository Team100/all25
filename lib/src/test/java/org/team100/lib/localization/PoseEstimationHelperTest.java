package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;

class PoseEstimationHelperTest {
    private static final boolean DEBUG = false;
    private static final double kDelta = 0.01;

    @Test
    void testGetRobotPoseInFieldCoords2() {
        // trivial example: if camera offset happens to match the camera global pose
        // then of course the robot global pose is the origin.
        Transform3d cameraInRobot = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInField = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));
        Transform3d tagInCamera = new Transform3d(new Translation3d(1, 0, 0), new Rotation3d());
        Pose3d cameraInField = PoseEstimationHelper.cameraInField(tagInField, tagInCamera);
        Pose3d robotInField = PoseEstimationHelper.robotInField(
                cameraInField,
                cameraInRobot);

        assertEquals(0, robotInField.getX(), kDelta);
        assertEquals(0, robotInField.getY(), kDelta);
        assertEquals(0, robotInField.getZ(), kDelta);
        assertEquals(0, robotInField.getRotation().getX(), kDelta);
        assertEquals(0, robotInField.getRotation().getY(), kDelta);
        assertEquals(0, robotInField.getRotation().getZ(), kDelta);
    }

    @Test
    void testGetRobotPoseInFieldCoords3() {
        Transform3d cameraInRobot = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInField = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));
        Translation3d tagTranslationInCamera = new Translation3d(1, 0, 0);
        Rotation3d tagRotationInCamera = new Rotation3d(0, 0, 0);
        Transform3d tagInCamera = new Transform3d(
                tagTranslationInCamera,
                tagRotationInCamera);
        Pose3d cameraInField = PoseEstimationHelper.cameraInField(tagInField, tagInCamera);
        Pose3d robotInField = PoseEstimationHelper.robotInField(cameraInField, cameraInRobot);
        assertEquals(0, robotInField.getX(), kDelta);
        assertEquals(0, robotInField.getY(), kDelta);
        assertEquals(0, robotInField.getZ(), kDelta);
        assertEquals(0, robotInField.getRotation().getX(), kDelta);
        assertEquals(0, robotInField.getRotation().getY(), kDelta);
        assertEquals(0, robotInField.getRotation().getZ(), kDelta);
    }

    @Test
    void testGetRobotPoseInFieldCoords4() {
        Transform3d cameraInRobot = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInField = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));

        Rotation3d cameraRotationInFieldCoords = new Rotation3d();

        // one meter range (Z forward)
        // pure tilt note we don't actually use this

        Blip24 blip = new Blip24(7,
                new Transform3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, 0)));

        Translation3d tagTranslationInCamera = blip.blipToTransform().getTranslation();
        Rotation3d tagRotationInCamera = PoseEstimationHelper.tagRotationInCamera(
                tagInField.getRotation(),
                cameraRotationInFieldCoords);
        Transform3d tagInCamera = new Transform3d(tagTranslationInCamera, tagRotationInCamera);
        Pose3d cameraInField = PoseEstimationHelper.cameraInField(tagInField, tagInCamera);

        Pose3d robotInField = PoseEstimationHelper.robotInField(
                cameraInField,
                cameraInRobot);
        assertEquals(0, robotInField.getX(), kDelta);
        assertEquals(0, robotInField.getY(), kDelta);
        assertEquals(0, robotInField.getZ(), kDelta);
        assertEquals(0, robotInField.getRotation().getX(), kDelta);
        assertEquals(0, robotInField.getRotation().getY(), kDelta);
        assertEquals(0, robotInField.getRotation().getZ(), kDelta);
    }

    @Test
    void testGetRobotPoseInFieldCoordsUsingCameraRotation24() {
        Transform3d cameraInRobot = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInField = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));

        // one meter range (Z forward)
        // identity rotation
        Blip24 blip = new Blip24(5,
                new Transform3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, 0)));

        Transform3d tagInCamera = blip.blipToTransform();

        Pose3d robotInField = PoseEstimationHelper.robotInField(
                cameraInRobot,
                tagInField,
                tagInCamera);

        assertEquals(0, robotInField.getX(), kDelta);
        assertEquals(0, robotInField.getY(), kDelta);
        assertEquals(0, robotInField.getZ(), kDelta);
        assertEquals(0, robotInField.getRotation().getX(), kDelta);
        assertEquals(0, robotInField.getRotation().getY(), kDelta);
        assertEquals(0, robotInField.getRotation().getZ(), kDelta);
    }

    @Test
    void testGetRobotPoseInFieldCoords524() {
        Transform3d cameraInRobot = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInField = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));

        // identity rotation
        // one meter range (Z forward)
        Blip24 blip = new Blip24(5,
                new Transform3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, 0)));

        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d();

        Transform3d tagInCamera = blip.blipToTransform();

        tagInCamera = PoseEstimationHelper.tagInCamera(
                cameraInRobot,
                tagInField,
                tagInCamera,
                robotRotationInFieldCoordsFromGyro);

        Pose3d robotPoseInField = PoseEstimationHelper.robotInField(
                cameraInRobot,
                tagInField,
                tagInCamera);

        assertEquals(0, robotPoseInField.getX(), kDelta);
        assertEquals(0, robotPoseInField.getY(), kDelta);
        assertEquals(0, robotPoseInField.getZ(), kDelta);
        assertEquals(0, robotPoseInField.getRotation().getX(), kDelta);
        assertEquals(0, robotPoseInField.getRotation().getY(), kDelta);
        assertEquals(0, robotPoseInField.getRotation().getZ(), kDelta);
    }

    /**
     * Is handling the camera input a performance issue?
     * 
     * On my desktop (i5-9400F at 2.9 Ghz), this test pegs one cpu core,
     * and takes about 270 ns per call. The roboRio 2.0 goes at 866 Mhz
     * so maybe 1/4 the speed, so this will take around 1 microsecond per call.
     * the total cycle budget is 20000 microseconds, though it would be good to
     * take much less than that; say 5000 microseconds. with a few cameras,
     * the total load would be, say, 0.1 percent of the budget.
     * 
     * So, not an issue.
     */
    // There's no need to run this all the time
    // @Test
    void posePerformance() {
        Transform3d cameraInRobot = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d tagInField = new Pose3d(2, 1, 1, new Rotation3d(0, 0, 0));

        // identity rotation
        // one meter range (Z forward)
        Blip24 blip = new Blip24(5,
                new Transform3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, 0)));

        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d();

        final int iterations = 10000000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            Transform3d tagInCamera = blip.blipToTransform();
            tagInCamera = PoseEstimationHelper.tagInCamera(
                    cameraInRobot,
                    tagInField,
                    tagInCamera,
                    robotRotationInFieldCoordsFromGyro);
        }
        long finishTime = System.currentTimeMillis();
        if (DEBUG) {
            Util.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
            Util.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
        }
    }

    @Test
    void testCameraRotationInFieldCoords() {
        Transform3d cameraInRobot = new Transform3d(
                new Translation3d(0.5, 0, 0.5), // front camera
                new Rotation3d(0, -Math.PI / 4, Math.PI / 4)); // pi/4 tilt up, pi/4 yaw left
        // robot rotation should only ever be yaw
        Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(0, 0, Math.PI / 2);
        Rotation3d cameraRotationInField = PoseEstimationHelper.cameraRotationInField(
                cameraInRobot,
                robotRotationInFieldCoordsFromGyro);
        assertEquals(0, cameraRotationInField.getX(), kDelta); // still no roll
        assertEquals(-Math.PI / 4, cameraRotationInField.getY(), kDelta); // same tilt
        assertEquals(3 * Math.PI / 4, cameraRotationInField.getZ(), kDelta);
    }

    @Test
    void testBlip24ToTransform() {
        { // identity
            Blip24 blip = new Blip24(5,
                    new Transform3d(
                            new Translation3d(),
                            new Rotation3d()));
            Transform3d transform3d = blip.blipToTransform();
            assertEquals(0, transform3d.getX(), kDelta);
            assertEquals(0, transform3d.getY(), kDelta);
            assertEquals(0, transform3d.getZ(), kDelta);
            assertEquals(0, transform3d.getRotation().getX(), kDelta);
            assertEquals(0, transform3d.getRotation().getY(), kDelta);
            assertEquals(0, transform3d.getRotation().getZ(), kDelta);
        }
        {
            Blip24 blip = new Blip24(5,
                    new Transform3d(
                            new Translation3d(-2, -1, 3),
                            new Rotation3d(Math.PI / 4, 0, 0)));
            Transform3d transform3d = blip.blipToTransform();
            assertEquals(3, transform3d.getX(), kDelta);
            assertEquals(2, transform3d.getY(), kDelta);
            assertEquals(1, transform3d.getZ(), kDelta);
            assertEquals(0, transform3d.getRotation().getX(), kDelta);
            assertEquals(-Math.PI / 4, transform3d.getRotation().getY(), kDelta);
            assertEquals(0, transform3d.getRotation().getZ(), kDelta);
        }
    }

    @Test
    void testBlip24ToTranslation() {
        // Blip is "z-forward", one meter up, two meters left, three meters ahead
        // rotation doesn't matter
        Blip24 blip = new Blip24(5,
                new Transform3d(
                        new Translation3d(-2, -1, 3),
                        new Rotation3d()));

        Translation3d nwuTranslation = blip.blipToTransform().getTranslation();

        // NWU values now
        assertEquals(3, nwuTranslation.getX(), kDelta);
        assertEquals(2, nwuTranslation.getY(), kDelta);
        assertEquals(1, nwuTranslation.getZ(), kDelta);
    }

    @Test
    void testBlip24ToRotation() {
        { // identity rotation
            Blip24 blip = new Blip24(5,
                    new Transform3d(
                            new Translation3d(-2, -1, 3),
                            new Rotation3d()));

            Rotation3d nwuRotation = blip.blipToTransform().getRotation();
            assertEquals(0, nwuRotation.getX(), kDelta);
            assertEquals(0, nwuRotation.getY(), kDelta);
            assertEquals(0, nwuRotation.getZ(), kDelta);
        }
        {
            // one meter range (Z forward)
            // tilt up in camera frame = +x rot
            Blip24 blip = new Blip24(5,
                    new Transform3d(
                            new Translation3d(0, Math.sqrt(2) / 2, Math.sqrt(2) / 2),
                            new Rotation3d(Math.PI / 4, 0, 0)));

            Rotation3d nwuRotation = blip.blipToTransform().getRotation();
            // tilt up in NWU is -y
            assertEquals(0, nwuRotation.getX(), kDelta);
            assertEquals(-Math.PI / 4, nwuRotation.getY(), kDelta);
            assertEquals(0, nwuRotation.getZ(), kDelta);
        }
        {
            // one meter range (Z forward)
            // pan right in camera frame = +y rot
            Blip24 blip = new Blip24(5,
                    new Transform3d(
                            new Translation3d(0, Math.sqrt(2) / 2, Math.sqrt(2) / 2),
                            new Rotation3d(0, Math.PI / 4, 0)));

            Rotation3d nwuRotation = blip.blipToTransform().getRotation();
            // pan right in NWU is -z
            assertEquals(0, nwuRotation.getX(), kDelta);
            assertEquals(0, nwuRotation.getY(), kDelta);
            assertEquals(-Math.PI / 4, nwuRotation.getZ(), kDelta);
        }
    }

    /**
     * We don't trust the camera's estimate of tag rotation. Instead, since we know
     * the pose of the tag, and we know the rotation of the camera (from the
     * gyro/compass and the camera offset), we calculate what the tag rotation
     * *should* be.
     */
    @Test
    void testtagRotationInRobotCoordsFromGyro() {
        {
            Rotation3d tagRotationInField = new Rotation3d(0, 0, 0); // on the far wall
            Rotation3d cameraRotationInField = new Rotation3d(0, 0, Math.PI / 4); // 45 degrees left
            Rotation3d tagRotationInCamera = PoseEstimationHelper.tagRotationInCamera(
                    tagRotationInField,
                    cameraRotationInField);
            assertEquals(0, tagRotationInCamera.getX(), kDelta);
            assertEquals(0, tagRotationInCamera.getY(), kDelta);
            assertEquals(-Math.PI / 4, tagRotationInCamera.getZ(), kDelta); // 45 degrees right
        }
        {
            Rotation3d tagRotationInField = new Rotation3d(0, 0, 0); // on the far wall
            Rotation3d cameraRotationInField = new Rotation3d(0, -Math.PI / 4, 0); // 45 degrees up
            Rotation3d tagRotationInCamera = PoseEstimationHelper.tagRotationInCamera(
                    tagRotationInField,
                    cameraRotationInField);
            assertEquals(0, tagRotationInCamera.getX(), kDelta);
            assertEquals(Math.PI / 4, tagRotationInCamera.getY(), kDelta); // 45 degrees down
            assertEquals(0, tagRotationInCamera.getZ(), kDelta);
        }
        {
            Rotation3d tagRotationInField = new Rotation3d(0, 0, 0); // on the far wall
            Rotation3d cameraRotationInField = new Rotation3d(0, -Math.PI / 4, Math.PI / 4); // pan/tilt
            Rotation3d tagRotationInCamera = PoseEstimationHelper.tagRotationInCamera(
                    tagRotationInField,
                    cameraRotationInField);
            // composing and then inverting multiple rotations yields this:
            assertEquals(-0.615, tagRotationInCamera.getX(), kDelta);
            assertEquals(0.523, tagRotationInCamera.getY(), kDelta);
            assertEquals(-0.955, tagRotationInCamera.getZ(), kDelta);
        }
    }

    @Test
    void testToFieldCoordinates() {
        {
            // opposite corner of 1,1 square
            Transform3d tagInCameraCords = new Transform3d(
                    new Translation3d(Math.sqrt(2), 0, 0),
                    new Rotation3d(0, 0, -Math.PI / 4));
            Pose3d tagInFieldCords = new Pose3d(
                    new Translation3d(1, 1, 0),
                    new Rotation3d());
            Pose3d cameraInField = PoseEstimationHelper.cameraInField(tagInFieldCords, tagInCameraCords);
            assertEquals(0, cameraInField.getX(), kDelta);
            assertEquals(0, cameraInField.getY(), kDelta);
            assertEquals(0, cameraInField.getZ(), kDelta);
            assertEquals(0, cameraInField.getRotation().getX(), kDelta);
            assertEquals(0, cameraInField.getRotation().getY(), kDelta);
            assertEquals(Math.PI / 4, cameraInField.getRotation().getZ(), kDelta); // pan45
        }
        {
            // in front, high
            Transform3d tagInCameraCords = new Transform3d(
                    new Translation3d(Math.sqrt(2), 0, 0),
                    new Rotation3d(0, Math.PI / 4, 0));
            Pose3d tagInFieldCords = new Pose3d(
                    new Translation3d(1, 0, 1),
                    new Rotation3d());
            Pose3d cameraInField = PoseEstimationHelper.cameraInField(tagInFieldCords, tagInCameraCords);
            assertEquals(0, cameraInField.getX(), kDelta);
            assertEquals(0, cameraInField.getY(), kDelta);
            assertEquals(0, cameraInField.getZ(), kDelta);
            assertEquals(0, cameraInField.getRotation().getX(), kDelta);
            assertEquals(-Math.PI / 4, cameraInField.getRotation().getY(), kDelta); // tilt45
            assertEquals(0, cameraInField.getRotation().getZ(), kDelta);
        }
    }

    @Test
    void testApplyCameraOffset() {
        // trivial example: if camera offset happens to match the camera global pose
        // then of course the robot global pose is the origin.
        Pose3d cameraInField = new Pose3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Transform3d cameraInRobot = new Transform3d(
                new Translation3d(1, 1, 1),
                new Rotation3d(0, 0, 0));
        Pose3d robotInField = PoseEstimationHelper.robotInField(cameraInField, cameraInRobot);
        assertEquals(0, robotInField.getX(), kDelta);
        assertEquals(0, robotInField.getY(), kDelta);
        assertEquals(0, robotInField.getZ(), kDelta);
        assertEquals(0, robotInField.getRotation().getX(), kDelta);
        assertEquals(0, robotInField.getRotation().getY(), kDelta);
        assertEquals(0, robotInField.getRotation().getZ(), kDelta);
    }

    @Test
    void testTagRotationIncorrect24() throws IOException {
        // this illustrates the WRONG WRONG WRONG tag orientation.

        // say we're playing blue, on the blue side, looking at tag 7.
        // the robot is facing 180, and the camera returns this.
        // note that the camera code returns the identity rotation when
        // it's looking straight at a tag, which implies "into the page"
        // orientation.

        Blip24 blip = new Blip24(7,
                new Transform3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, 0)));

        Transform3d tagInCamera = blip.blipToTransform();
        assertEquals(1, tagInCamera.getX(), kDelta);
        assertEquals(0, tagInCamera.getY(), kDelta);
        assertEquals(0, tagInCamera.getZ(), kDelta);
        assertEquals(0, tagInCamera.getRotation().getX(), kDelta);
        assertEquals(0, tagInCamera.getRotation().getY(), kDelta);
        assertEquals(0, tagInCamera.getRotation().getZ(), kDelta);

        // "raw" layout, which is "out of the page" tag orientation.j
        // which is WRONG WRONG WRONG
        Path path = Filesystem.getDeployDirectory().toPath().resolve("2025-reefscape.json");
        AprilTagFieldLayout layout = new AprilTagFieldLayout(path);
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        Pose3d tagInField = layout.getTagPose(7).get();

        Pose3d cameraInField = PoseEstimationHelper.cameraInField(tagInField, tagInCamera);

        // notice this is WRONG WRONG WRONG because the raw tag rotation is also WRONG
        assertEquals(12.89, cameraInField.getX(), kDelta);
        assertEquals(4.026, cameraInField.getY(), kDelta);
        assertEquals(0.308, cameraInField.getZ(), kDelta);
        assertEquals(0, cameraInField.getRotation().getX(), kDelta);
        assertEquals(0, cameraInField.getRotation().getY(), kDelta);
        // camera is facing down field which is WRONG WRONG WRONG
        assertEquals(0, cameraInField.getRotation().getZ(), kDelta);
    }

    @Test
    void testTagRotationCorrect24() throws IOException {
        // this illustrates the CORRECT tag orientation.

        // say we're playing blue, on the blue side, looking at tag 7.
        // the robot is facing 180, and the camera returns this.
        // note that the camera code returns the identity rotation when
        // it's looking straight at a tag, which implies "into the page"
        // orientation.

        Blip24 blip = new Blip24(7,
                new Transform3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, 0)));

        Transform3d tagInCamera = blip.blipToTransform();
        assertEquals(1, tagInCamera.getX(), kDelta);
        assertEquals(0, tagInCamera.getY(), kDelta);
        assertEquals(0, tagInCamera.getZ(), kDelta);
        assertEquals(0, tagInCamera.getRotation().getX(), kDelta);
        assertEquals(0, tagInCamera.getRotation().getY(), kDelta);
        assertEquals(0, tagInCamera.getRotation().getZ(), kDelta);

        // first try the "corrected" layout, which is "into the page" tag orientation.
        // this is CORRECT
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        Pose3d tagInFieldCoords = layout.getTagPose(Alliance.Blue, 7).get();

        Pose3d cameraInField = PoseEstimationHelper.cameraInField(tagInFieldCoords, tagInCamera);

        // the tag is a little bit behind the line, so we're a little closer to the line
        // than 1m.
        assertEquals(14.890, cameraInField.getX(), kDelta);
        // the tag is over to the left; so is the camera
        assertEquals(4.026, cameraInField.getY(), kDelta);
        // tag center is about 57 inches up; so is the camera
        assertEquals(0.308, cameraInField.getZ(), kDelta);
        // zero roll
        assertEquals(0, cameraInField.getRotation().getX(), kDelta);
        // zero tilt
        assertEquals(0, cameraInField.getRotation().getY(), kDelta);
        // camera is facing back towards the wall
        assertEquals(Math.PI, cameraInField.getRotation().getZ(), kDelta);
    }
}
