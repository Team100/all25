package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** These tests use the 2025 map, will have to be updated in 2026. */
public class AprilTagFieldLayoutWithCorrectOrientationTest {
    private static final double kDelta = 0.001;

    @Test
    void testGetTagPoseRed() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d pose = layout.getTagPose(Alliance.Red, 1).get();
        // tag 1 coordinates for red
        assertEquals(0.851, pose.getX(), kDelta);
        assertEquals(7.396, pose.getY(), kDelta);
        assertEquals(1.486, pose.getZ(), kDelta);
    }

    @Test
    void testGetPoseBlue() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d pose = layout.getTagPose(Alliance.Blue, 1).get();
        // tag 1 coordinates for blue
        assertEquals(16.697, pose.getX(), kDelta);
        assertEquals(0.655, pose.getY(), kDelta);
        assertEquals(1.486, pose.getZ(), kDelta);
    }
}
