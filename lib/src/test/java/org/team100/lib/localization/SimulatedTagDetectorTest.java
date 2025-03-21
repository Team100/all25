package org.team100.lib.localization;

import java.io.IOException;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Camera;

import edu.wpi.first.math.geometry.Pose2d;

public class SimulatedTagDetectorTest {
    @Test
    void testSimple() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        SimulatedTagDetector sim = new SimulatedTagDetector(
                List.of(Camera.CORAL_LEFT),
                layout,
                () -> new Pose2d());
        sim.periodic();
    }
}
