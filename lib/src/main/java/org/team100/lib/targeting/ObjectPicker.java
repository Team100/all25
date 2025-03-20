package org.team100.lib.targeting;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ObjectPicker {

    /**
     * Given a list of field-relative translations, return the one closest to the
     * robot.
     * 
     * @param notes     the field relative pose of detected notes
     * @param robotPose the pose of the swerve drivetrain
     * @return The field relative translation of the closest note, or empty if none
     *         nearby
     */
    public static Optional<Translation2d> closestObject(
            List<Translation2d> notes,
            Pose2d robotPose) {
        if (notes.isEmpty()) {
            return Optional.empty();
        }
        Translation2d robotTranslation = robotPose.getTranslation();
        double shortestDistance = 1000000000;
        Optional<Translation2d> closestTranslation = Optional.empty();
        for (Translation2d note : notes) {
            // if (note.getY() < -1 || note.getX() < -1 || note.getY() > 9.21 || note.getX()
            // > 17.54) {
            // // ignore out-of-bounds
            // continue;
            // }
            double distance = robotTranslation.getDistance(note);
            if (distance < shortestDistance) {
                shortestDistance = distance;
                closestTranslation = Optional.of(note);
            }
        }
        return closestTranslation;
    }

    private ObjectPicker() {
        //
    }
}
