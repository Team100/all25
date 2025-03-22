package org.team100.lib.localization;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.StringLogger;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Static methods used to interpret camera input.
 */
public class PoseEstimationHelper implements Glassy {
    private static final boolean DEBUG = false;

    private final StringLogger m_log_rotation_source;

    public PoseEstimationHelper(LoggerFactory parent) {
        LoggerFactory child = parent.child(this);
        m_log_rotation_source = child.stringLogger(Level.TRACE, "rotation_source");
    }

    /**
     * Calculate robot pose.
     * 
     * First calculates the distance to the tag. If it's closer than the threshold,
     * use the camera-derived tag rotation. If it's far, use the gyro.
     */
    public Pose3d getRobotPoseInFieldCoords(
            Transform3d cameraInRobot,
            Pose3d tagInField,
            Transform3d tagInCamera,
            Rotation3d robotRotationInFieldFromGyro,
            double thresholdMeters) {
        if (tagInCamera.getTranslation().getNorm() < thresholdMeters) {
            m_log_rotation_source.log(() -> "CAMERA");
            return getRobotPoseInFieldCoords(
                    cameraInRobot,
                    tagInField,
                    tagInCamera);
        }
        m_log_rotation_source.log(() -> "GYRO");
        return getRobotPoseInFieldCoords(
                cameraInRobot,
                tagInField,
                tagInCamera,
                robotRotationInFieldFromGyro);
    }

    /**
     * Calculate robot pose.
     * 
     * Given the blip and its corresponding field location, and the camera offset,
     * return the robot pose in field coordinates.
     * 
     * This method trusts the tag rotation calculated by the camera.
     */
    public static Pose3d getRobotPoseInFieldCoords(
            Transform3d cameraInRobot,
            Pose3d tagInField,
            Transform3d tagInCamera) {

        if (DEBUG) {
            // This is used for camera offset calibration. Place a tag at a known position,
            // set the tag rotation belief threshold to a very high number (so this code
            // executes), observe the offset, and add it to Camera.java, inverted.
            Transform3d tagInRobot = cameraInRobot.plus(tagInCamera);
            Util.printf("TAG IN ROBOT %s x %f y %f z%f\n",
                    tagInRobot.getTranslation(),
                    tagInRobot.getRotation().getX(),
                    tagInRobot.getRotation().getY(),
                    tagInRobot.getRotation().getZ());
        }
        Pose3d cameraInField = toFieldCoordinates(tagInCamera, tagInField);
        return applyCameraOffset(cameraInField, cameraInRobot);
    }

    /**
     * Calculate robot pose.
     * 
     * Given the blip, the heading, the camera offset, and the absolute tag pose,
     * return the absolute robot pose in field coordinates.
     * 
     * This method does not trust the tag rotation from the camera, it uses the gyro
     * signal instead.
     * 
     * @param cameraInRobot                camera offset expressed as a
     *                                           transform from robot-frame to
     *                                           camera-frame, e.g.camera 0.5m in
     *                                           front of the robot center and 0.5m
     *                                           from the floor would have a
     *                                           translation (0.5, 0, 0.5)
     * @param tagInField                   tag location expressed as a pose in
     *                                           field-frame. this should come from
     *                                           the json
     * @param apparentTagInCamera          Tag pose in camera coordinates; we
     *                                           ignore the rotational component of
     *                                           this pose.
     * @param robotRotationInFieldCoordsFromGyro direct from the gyro. note that
     *                                           drive.getPose() isn't exactly the
     *                                           gyro reading; it might be better to
     *                                           use the real gyro than the getPose
     *                                           method.
     */
    public Pose3d getRobotPoseInFieldCoords(
            Transform3d cameraInRobot,
            Pose3d tagInField,
            Transform3d apparentTagInCamera,
            Rotation3d robotRotationInFieldCoordsFromGyro) {

        Rotation3d cameraRotationInFieldCoords = cameraRotationInFieldCoords(
                cameraInRobot,
                robotRotationInFieldCoordsFromGyro);

        Translation3d tagTranslationInCameraCoords = apparentTagInCamera.getTranslation();

        Rotation3d tagRotationInCameraCoords = tagRotationInRobotCoordsFromGyro(
                tagInField.getRotation(),
                cameraRotationInFieldCoords);

        Transform3d tagInCameraCoords = new Transform3d(
                tagTranslationInCameraCoords,
                tagRotationInCameraCoords);

        Pose3d cameraInFieldCoords = toFieldCoordinates(
                tagInCameraCoords,
                tagInField);

        return applyCameraOffset(
                cameraInFieldCoords,
                cameraInRobot);
    }

    //////////////////////////////
    //
    // package private below, don't use these.

    /**
     * given the gyro rotation and the camera offset, return the camera absolute
     * rotation. Package-private for testing.
     */
    static Rotation3d cameraRotationInFieldCoords(
            Transform3d cameraInRobot,
            Rotation3d robotRotationInFieldFromGyro) {
        return cameraInRobot.getRotation().rotateBy(robotRotationInFieldFromGyro);
    }

    /**
     * Because the camera's estimate of tag rotation isn't very accurate, this
     * synthesizes an estimate using the tag rotation in field frame (from json) and
     * the camera rotation in field frame (from gyro). Package-private for testing.
     */
    static Rotation3d tagRotationInRobotCoordsFromGyro(
            Rotation3d tagRotationInField,
            Rotation3d cameraRotationInField) {
        return tagRotationInField.rotateBy(cameraRotationInField.unaryMinus());
    }

    /**
     * Given the tag in camera frame and tag in field frame, return the camera in
     * field frame. Package-private for testing.
     */
    static Pose3d toFieldCoordinates(Transform3d tagInCamera, Pose3d tagInField) {
        // First invert the camera-to-tag transform, obtaining tag-to-camera.
        Transform3d cameraInTag = tagInCamera.inverse();
        // Then compose field-to-tag with tag-to-camera to get field-to-camera.
        return tagInField.transformBy(cameraInTag);
    }

    /**
     * Given the camera in field frame and camera in robot frame, return the robot
     * in field frame. Package-private for testing.
     */
    static Pose3d applyCameraOffset(Pose3d cameraInField, Transform3d cameraInRobot) {
        Transform3d robotInCamera = cameraInRobot.inverse();
        return cameraInField.transformBy(robotInCamera);
    }
}
