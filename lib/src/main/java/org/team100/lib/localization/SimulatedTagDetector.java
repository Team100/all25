package org.team100.lib.localization;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.config.Camera;
import org.team100.lib.util.Util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Publishes AprilTag Blip24 sightings on Network Tables, just like real
 * cameras would.
 */
public class SimulatedTagDetector {
    private static final boolean DEBUG = true;

    private final List<Camera> m_cameras;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    private final Supplier<Pose2d> m_robotPose;

    public SimulatedTagDetector(
            List<Camera> cameras,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Supplier<Pose2d> robotPose) {
        m_cameras = cameras;
        m_layout = layout;
        m_robotPose = robotPose;
    }

    public void periodic() {
        if (DEBUG)
            Util.println("simulated tag detector");
        Optional<Alliance> opt = DriverStation.getAlliance();
        if (opt.isEmpty())
            return;
        Pose3d robotPose3d = new Pose3d(m_robotPose.get());
        for (Camera camera : m_cameras) {
            Transform3d offset = camera.getOffset();
            Pose3d cameraPose3d = robotPose3d.plus(offset);
            for (AprilTag tag : m_layout.getTags(opt.get())) {
                Pose3d tagPose = tag.pose;
                Transform3d tagInCamera = tagPose.minus(cameraPose3d);
                if (DEBUG)
                    Util.printf("camera %s offset %s tag %d in camera %s\n", camera.name(), offset, tag.ID,
                            tagInCamera);
            }

        }

    }

}
