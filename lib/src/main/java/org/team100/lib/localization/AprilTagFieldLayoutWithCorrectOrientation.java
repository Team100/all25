package org.team100.lib.localization;

import java.io.IOException;
import java.nio.file.Path;
import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * The WPILib JSON tag file, and the wrapper, AprilTagFieldLayout, define tag
 * rotation with respect to the *outward* normal, which is the opposite of the
 * Apriltags convention to use the *inward* normal.
 * 
 * This wrapper "fixes" the orientations so they match the Apriltag convention,
 * and thus match the result of camera pose estimates. Without this fix, we
 * would have to sprinkle inversions here and there, which would result in bugs.
 * 
 * In 2024 the pose returned from the camera has zero rotation when facing the
 * tag, which corresponds to the "inward normal" orientation.
 * 
 * The 2024 game map retains the "outward normal" orientation, and we're using
 * the WPILib wrapper around the Apriltag library, which does NOT invert the
 * canonical tag orientation.
 * 
 * The 2025 game map is the same.
 * 
 * AprilTagPoseEstimator.cpp seems to wrap the apriltag library, and then
 * transform the returned pose array into WPI the domain object with no
 * adjustment (i.e. it uses the raw rotation matrix, orthogonalized)
 * 
 * @see https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#coordinate-system
 * 
 * NOTE: the AprilTag object is just the raw JSON, not corrected for alliance
 * orientation.  Do not use the AprilTag object!
 */
public class AprilTagFieldLayoutWithCorrectOrientation {
    private static final String kProdFilename = "2025-reefscape.json";

    // Inverts yaw
    private static final Transform3d kFix = new Transform3d(
            new Translation3d(),
            new Rotation3d(0, 0, Math.PI));

    private final Map<Alliance, AprilTagFieldLayout> layouts = new EnumMap<>(Alliance.class);

    public AprilTagFieldLayoutWithCorrectOrientation() throws IOException {
        Path path = Filesystem.getDeployDirectory().toPath().resolve(kProdFilename);

        AprilTagFieldLayout blueLayout = new AprilTagFieldLayout(path);
        blueLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        AprilTagFieldLayout redLayout = new AprilTagFieldLayout(path);
        redLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);

        layouts.put(Alliance.Red, redLayout);
        layouts.put(Alliance.Blue, blueLayout);
    }

    /**
     * @return Tag pose with correct yaw (inverted compared to json file)
     */
    public Optional<Pose3d> getTagPose(Alliance alliance, int id) {
        AprilTagFieldLayout layout = layouts.get(alliance);
        Optional<Pose3d> pose = layout.getTagPose(id);
        if (pose.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(pose.get().transformBy(kFix));
    }
}
