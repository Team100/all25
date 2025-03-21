package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team100.lib.config.Camera;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class SimulatedTagDetectorTest {
    private static final double kDelta = 0.001;

    @BeforeEach
    void init() {
        HAL.initialize(500, 0);
    }

    @Test
    void testSimple() throws IOException {
        List<Camera> cameras = List.of(
                Camera.SWERVE_LEFT,
                Camera.SWERVE_RIGHT,
                Camera.FUNNEL,
                Camera.CORAL_LEFT,
                Camera.CORAL_RIGHT);
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        // right in front of tag 7
        Pose2d pose = new Pose2d(2.6576, 4.0259, Rotation2d.kZero);
        SimulatedTagDetector sim = new SimulatedTagDetector(cameras, layout, () -> pose);
        // sim uses alliance from driver station
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();
        sim.periodic();
    }

    @Test
    void testProjection1() {
        // camera at origin
        Pose3d cameraPose3d = new Pose3d();
        // tag in front
        Pose3d tagPose = new Pose3d(1, 0, 0, new Rotation3d());
        Transform3d tagInCamera = SimulatedTagDetector.tagInCamera(cameraPose3d, tagPose);
        assertEquals(1, tagInCamera.getTranslation().getX(), kDelta);
        assertEquals(0, tagInCamera.getTranslation().getY(), kDelta);
        assertEquals(0, tagInCamera.getTranslation().getZ(), kDelta);
        assertEquals(0, tagInCamera.getRotation().getX(), kDelta); // roll
        assertEquals(0, tagInCamera.getRotation().getY(), kDelta); // pitch
        assertEquals(0, tagInCamera.getRotation().getZ(), kDelta); // yaw
    }

    @Test
    void testProjection2() {
        // camera offset in y
        Pose3d cameraPose3d = new Pose3d(0, 1, 0, new Rotation3d());
        // tag in front
        Pose3d tagPose = new Pose3d(1, 0, 0, new Rotation3d());
        Transform3d tagInCamera = SimulatedTagDetector.tagInCamera(cameraPose3d, tagPose);
        assertEquals(1, tagInCamera.getTranslation().getX(), kDelta);
        assertEquals(-1, tagInCamera.getTranslation().getY(), kDelta);
        assertEquals(0, tagInCamera.getTranslation().getZ(), kDelta);
        assertEquals(0, tagInCamera.getRotation().getX(), kDelta); // roll
        assertEquals(0, tagInCamera.getRotation().getY(), kDelta); // pitch
        assertEquals(0, tagInCamera.getRotation().getZ(), kDelta); // yaw
    }

    @Test
    void testProjection3() {
        // camera pan left 45
        Pose3d cameraPose3d = new Pose3d(0, 0, 0, new Rotation3d(0, 0, Math.PI / 4));
        // tag in front
        Pose3d tagPose = new Pose3d(1, 0, 0, new Rotation3d());
        Transform3d tagInCamera = SimulatedTagDetector.tagInCamera(cameraPose3d, tagPose);
        assertEquals(0.707, tagInCamera.getTranslation().getX(), kDelta);
        assertEquals(-0.707, tagInCamera.getTranslation().getY(), kDelta);
        assertEquals(0, tagInCamera.getTranslation().getZ(), kDelta);
        assertEquals(0, tagInCamera.getRotation().getX(), kDelta); // roll
        assertEquals(0, tagInCamera.getRotation().getY(), kDelta); // pitch
        assertEquals(-Math.PI / 4, tagInCamera.getRotation().getZ(), kDelta); // yaw
    }

    @Test
    void testProjection4() {
        // camera pan left 45
        Pose3d cameraPose3d = new Pose3d(0, 0, 0, new Rotation3d(0, 0, Math.PI / 4));
        // tag rotated
        Pose3d tagPose = new Pose3d(1, 0, 1, new Rotation3d(1, 2, 3));
        Transform3d tagInCamera = SimulatedTagDetector.tagInCamera(cameraPose3d, tagPose);
        // position is not affectted by tag rotation
        assertEquals(0.707, tagInCamera.getTranslation().getX(), kDelta);
        assertEquals(-0.707, tagInCamera.getTranslation().getY(), kDelta);
        assertEquals(1, tagInCamera.getTranslation().getZ(), kDelta);
        // tag rotation is transformed
        assertEquals(-2.142, tagInCamera.getRotation().getX(), kDelta); // roll
        assertEquals(1.142, tagInCamera.getRotation().getY(), kDelta); // pitch
        assertEquals(-0.927, tagInCamera.getRotation().getZ(), kDelta); // yaw
    }

    @Test
    void testFrustum() {
        // normalized coordinates here are (1,1), which is outside the FOV.
        assertFalse(SimulatedTagDetector.visible(
                new Transform3d(new Translation3d(1, 1, 1), new Rotation3d())));

        // normalized coordinates are (0.5, 0.5) which is visible.
        assertTrue(SimulatedTagDetector.visible(
                new Transform3d(new Translation3d(1, 0.5, 0.5), new Rotation3d())));

        // frustum faces forwards only.
        assertFalse(SimulatedTagDetector.visible(
                new Transform3d(new Translation3d(-1, 0.5, 0.5), new Rotation3d())));
    }

    @Test
    void testFrontBack() {
        // how to tell if the tag is facing us?
        // here's a sample, tag 10 seen from the back
        Transform3d tag10InCamera = new Transform3d(
                new Translation3d(2.20, 0.26, 1.22),
                new Rotation3d(0.04, 0.71, -2.89));
        // what's the angle between the rotation here and 1,0,0 (the camera bore)?
        Rotation3d tag10RotationInCamera = tag10InCamera.getRotation();
        // this is the total rotation, we don't care what the axis of rotation is,
        // if the tag rotates around *any* axis more than pi/2, it's not visible.
        double angle = tag10RotationInCamera.getAngle();
        assertEquals(2.92, angle, kDelta);

        // facing away is invisible
        assertFalse(SimulatedTagDetector.visible(
                new Transform3d(new Translation3d(1, 0, 0), new Rotation3d(0, 0, 2))));
    }

    @Test
    void testVisible1() {
        Rotation3d tag6 = new Rotation3d(0.62, -0.36, -0.93);
        System.out.println(tag6);
        // this case
        // Rotation3d(Quaternion(0.861, 0.191, -0.286, -0.371))
        // from above
        // Rotation3d(Quaternion(-0.862, -0.192, 0.286, 0.369))
        double angle = tag6.getAngle();
        assertEquals(1.063, angle, kDelta);

        // this seems reasonable
        Rotation3d thiscase = new Rotation3d(new Quaternion(0.861, 0.191, -0.286, -0.371));
        assertEquals(1.062, thiscase.getAngle(), kDelta);
        // this seems very wrong
        Rotation3d othercase = new Rotation3d(new Quaternion(-0.862, -0.192, 0.286, 0.369));
        assertEquals(5.224, othercase.getAngle(), kDelta);
    }

    @Test
    void testTag6() throws IOException {
        Camera camera = Camera.CORAL_LEFT;
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        // right in front of tag 7
        Pose2d robotPose = new Pose2d(2.6576, 4.0259, Rotation2d.kZero);
        Pose3d robotPose3d = new Pose3d(robotPose);

        Transform3d cameraOffset = camera.getOffset();
        Pose3d cameraPose3d = robotPose3d.plus(cameraOffset);
        // camera is in the front
        assertEquals(2.855, cameraPose3d.getTranslation().getX(), kDelta);
        // a little to the left
        assertEquals(4.31, cameraPose3d.getTranslation().getY(), kDelta);
        assertEquals(0.811, cameraPose3d.getTranslation().getZ(), kDelta);
        // rolled a little due to the mounting
        assertEquals(-0.158, cameraPose3d.getRotation().getX(), kDelta);
        // pitched down about 40 degrees
        assertEquals(0.691, cameraPose3d.getRotation().getY(), kDelta);
        // panned right about 17 degrees
        assertEquals(-0.295, cameraPose3d.getRotation().getZ(), kDelta);

        Pose3d tagPose = layout.getTagPose(Alliance.Red, 6).get();
        Transform3d tagInCamera = new Transform3d(cameraPose3d, tagPose);

        Translation3d tagTranslationInCamera = tagInCamera.getTranslation();
        Rotation3d tagRotationInCamera = tagInCamera.getRotation();

        // tag is ahead
        assertEquals(1.121, tagTranslationInCamera.getX(), kDelta);
        // to the left
        assertEquals(0.718, tagTranslationInCamera.getY(), kDelta);
        // a little above bore
        assertEquals(0.392, tagTranslationInCamera.getZ(), kDelta);
        // tag seems rolled
        assertEquals(0.621, tagRotationInCamera.getX(), kDelta);
        // tag seems pitched down
        assertEquals(-0.36, tagRotationInCamera.getY(), kDelta);
        // tag seems yawed right
        assertEquals(-0.926, tagRotationInCamera.getZ(), kDelta);
        // why is this so large?
        assertEquals(5.222, tagRotationInCamera.getAngle(), kDelta);

        Translation3d normal = new Translation3d(1, 0, 0);
        // this points "into the page" of the tag
        Translation3d rotatedNormal = normal.rotateBy(tagRotationInCamera);
        Vector<N3> rotatedNormalVector = rotatedNormal.toVector();
        Vector<N3> tagTranslationVector = tagTranslationInCamera.toVector();
        Rotation3d apparentAngle = new Rotation3d(tagTranslationVector, rotatedNormalVector);
        double angle = apparentAngle.getAngle();
        assertEquals(1.403, angle, kDelta);

    }
}
