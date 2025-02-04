package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Remind myself what's in the JSON file.
 * These are now inverted using the wrapper.
 */
class TagTest {
    private static final double kDelta = 0.01;

    @Test
    void testBlueLayout() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        /*
         * from the blue perspective, tag 7 has small x
         * and large y, and oriented at pi theta.
         */
        Pose3d tag7Pose = layout.getTagPose(Alliance.Blue, 7).get();
        assertEquals(13.89, tag7Pose.getTranslation().getX(), kDelta);
        assertEquals(4.026, tag7Pose.getTranslation().getY(), kDelta);
        assertEquals(0.308, tag7Pose.getTranslation().getZ(), kDelta);
        assertEquals(0, tag7Pose.getRotation().getX(), kDelta);
        assertEquals(0, tag7Pose.getRotation().getY(), kDelta);
        // "into the page" means facing towards the baseline, 180 degrees
        assertEquals(Math.PI, tag7Pose.getRotation().getZ(), kDelta);
    }

    @Test
    void testRedLayout() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        /*
         * from the red perspective, tag 7 has large x
         * and small y, and oriented at zero theta.
         */
        Pose3d tag7Pose = layout.getTagPose(Alliance.Red, 7).get();
        assertEquals(3.657, tag7Pose.getTranslation().getX(), kDelta);
        assertEquals(4.026, tag7Pose.getTranslation().getY(), kDelta); // close to right side
        assertEquals(0.308, tag7Pose.getTranslation().getZ(), kDelta); // 1.5m up (as above)
        assertEquals(0, tag7Pose.getRotation().getX(), kDelta);
        assertEquals(0, tag7Pose.getRotation().getY(), kDelta);
        // "into the page" i.e. away from the baseline, i.e. zero degrees
        assertEquals(0, tag7Pose.getRotation().getZ(), kDelta);
    }

    @Test
    void testRaw() throws IOException {
        Path path = Filesystem.getDeployDirectory().toPath().resolve("2025-reefscape.json");
        AprilTagFieldLayout layout = new AprilTagFieldLayout(path);
        // blue side, tag seven
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        Pose3d tag7Pose = layout.getTagPose(7).get();

        // on our side, x is ~zero.
        assertEquals(13.89, tag7Pose.getTranslation().getX(), kDelta); // behind the glass
        assertEquals(4.026, tag7Pose.getTranslation().getY(), kDelta); // far to left
        assertEquals(0.308, tag7Pose.getTranslation().getZ(), kDelta); // 1.5m feet up

        assertEquals(0, tag7Pose.getRotation().getX(), kDelta);
        assertEquals(0, tag7Pose.getRotation().getY(), kDelta);
        // raw rotation is "out of the page"
        assertEquals(0, tag7Pose.getRotation().getZ(), kDelta);
    }
}
