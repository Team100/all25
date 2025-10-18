package org.team100.lib.targeting;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class TargetLocalizer {
    private static final boolean DEBUG = false;
    private static final double OBJ_GROUND_OFFSET = 0.12;

    /**
     * Converts camera-relative sights to field relative translations.
     * 
     * NOTE! camera sights are x-ahead WPI coordinates, not z-ahead camera
     * coordinates.
     */
    public static List<Translation2d> cameraRotsToFieldRelativeArray(
            Pose2d robotPose,
            Transform3d cameraInRobotCoordinates,
            Rotation3d[] sights) {
        if (DEBUG) {
            System.out.print("camera rots\n");
        }

        ArrayList<Translation2d> Tnotes = new ArrayList<>();
        for (Rotation3d note : sights) {
            if (DEBUG) {
                System.out.printf("rotation %s\n", note);
            }
            cameraRotToFieldRelative(
                    robotPose,
                    cameraInRobotCoordinates,
                    note).ifPresent(Tnotes::add);
        }
        return Tnotes;
    }

    public static Optional<Translation2d> cameraRotToFieldRelative(
            Pose2d robotPose,
            Transform3d cameraInRobotCoordinates,
            Rotation3d note) {
        //
        // this appears to have filtered out very close poses, which i think is the
        // opposite of what it is supposed to do: avoid the horizon.
        // i'm kinda suspicious how this ever worked. there was no unit test
        // for it as of Jul 2024.
        // if (note.getY() < cameraInRobotCoordinates.getRotation().getY()) {
        //
        Optional<Translation2d> robotRelative = TargetLocalizer
                .sightToRobotRelative(cameraInRobotCoordinates, note);
        if (robotRelative.isEmpty()) {
            if (DEBUG) {
                System.out.print("empty\n");
            }
            return Optional.empty();
        }
        return Optional.of(TargetLocalizer.robotRelativeToFieldRelative(
                robotPose,
                robotRelative.get()));
    }

    /**
     * Return the robot-relative intersection of the camera-relative target sight
     * with the floor.
     */
    public static Optional<Translation2d> sightToRobotRelative(
            Transform3d cameraInRobotCoordinates,
            Rotation3d sight) {
        return TargetLocalizer.sightInRobotCoordsToTranslation2d(
                TargetLocalizer.sightInRobotCoords(cameraInRobotCoordinates, sight));
    }

    /**
     * Convert a camera-relative sight (with no translational component) into a
     * robot-relative sight.
     */
    public static Transform3d sightInRobotCoords(
            Transform3d cameraInRobotCoordinates,
            Rotation3d sight) {
        Transform3d sightTransform = new Transform3d(0, 0, 0, sight);
        return cameraInRobotCoordinates.plus(sightTransform);
    }

    /**
     * Return the robot-relative intersection of the robot-relative target sight
     * with the floor.
     */
    public static Optional<Translation2d> sightInRobotCoordsToTranslation2d(
            Transform3d robotRelativeSight) {
        double h = robotRelativeSight.getZ()-OBJ_GROUND_OFFSET;
        if (h <= 0) {
            if (DEBUG) {
                System.out.printf("camera is below the floor %f\n", h);
            }
            return Optional.empty();
        }
        double yaw = robotRelativeSight.getRotation().getZ();
        double pitch = robotRelativeSight.getRotation().getY();
        if (pitch <= 0) {
            if (DEBUG) {
                System.out.printf("target is above the horizon %f\n", pitch);
            }
            return Optional.empty();
        }
        double d = h / Math.tan(pitch);
        double x = robotRelativeSight.getX();
        double y = robotRelativeSight.getY();
        return Optional.of(new Translation2d(
                x + d * Math.cos(yaw),
                y + d * Math.sin(yaw)));
    }

    private TargetLocalizer() {
        //
    }

    /** Convert robot-relative translation to field-relative translation. */
    public static Translation2d robotRelativeToFieldRelative(
            Pose2d currentPose,
            Translation2d robotRelative) {
        return currentPose
                .transformBy(new Transform2d(robotRelative, new Rotation2d()))
                .getTranslation();
    }
}
