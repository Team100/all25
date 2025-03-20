package org.team100.lib.targeting;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.team100.lib.field.FieldPoint2024;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * This was from 2024.
 */
public class SimulatedObjectDetector {
    private final Transform3d m_offset;
    private final double m_hFovHalfAngleRad;
    private final double m_vFovHalfAngleRad;

    /**
     * @param offset           robot-relative
     * @param hFovHalfAngleRad horizontal field of view, half-angle
     * @param vFovHalfAngleRad vertical field of view, half-angle
     */
    public SimulatedObjectDetector(
            Transform3d offset,
            double hFovHalfAngleRad,
            double vFovHalfAngleRad) {
        m_offset = offset;
        m_hFovHalfAngleRad = hFovHalfAngleRad;
        m_vFovHalfAngleRad = vFovHalfAngleRad;
    }

    public Transform3d getOffset() {
        return m_offset;
    }

    public List<Rotation3d> getKnownLocations(Alliance alliance, Pose2d robotPose) {
        return getRotations(robotPose, FieldPoint2024.allNotes(alliance));
    }

    /**
     * Gets the rotation to the object in the frame
     * 
     * @param robotPose Pose of the robot
     * @param notes     field relative translation of any objects
     */
    public List<Rotation3d> getRotations(Pose2d robotPose, Translation2d[] notes) {
        ArrayList<Rotation3d> list = new ArrayList<>();
        for (Translation2d note : notes) {
            getRotInCamera(robotPose, note).ifPresent(list::add);
        }
        return list;
    }

    // package-private below for testing

    /**
     * Return the camera-relative rotation for the note, given the robot pose.
     */
    Optional<Rotation3d> getRotInCamera(Pose2d robotPose, Translation2d note) {
        Transform3d noteInCameraCoordinates = getNoteInCameraCoordinates(robotPose, note);
        double x = noteInCameraCoordinates.getX();
        double y = noteInCameraCoordinates.getY();
        double z = noteInCameraCoordinates.getZ();
        if (Math.abs(Math.atan2(z, x)) >= m_vFovHalfAngleRad
                && Math.abs(Math.atan2(y, x)) >= m_hFovHalfAngleRad) {
            return Optional.empty();
        }
        return Optional.of(new Rotation3d(VecBuilder.fill(x, 0, 0), VecBuilder.fill(x, y, z)));
    }

    /**
     * Find the note transform relative to the camera, given the field-relative
     * robot pose and note translation
     */
    Transform3d getNoteInCameraCoordinates(Pose2d robotPose, Translation2d note) {
        Pose2d notePose = new Pose2d(note, new Rotation2d());
        Translation2d relative = notePose.relativeTo(robotPose).getTranslation();
        return getNoteInCameraCoordinates(relative);
    }

    /**
     * Given a translation on the floor relative to the robot, return the transform
     * relative to the camera.
     */
    Transform3d getNoteInCameraCoordinates(Translation2d relative) {
        Transform3d noteInRobotCoords = new Transform3d(
                new Translation3d(relative.getX(), relative.getY(), 0),
                new Rotation3d());
        Transform3d robotInCameraCoordinates = m_offset.inverse();
        return robotInCameraCoordinates.plus(noteInRobotCoords);
    }
}
