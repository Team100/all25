package org.team100.lib.wpi_trajectory;

import static org.junit.jupiter.api.Assertions.assertThrows;

import java.io.IOException;
import java.util.List;
import java.util.NoSuchElementException;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

class TestMalformedSpline {
    @Test
    void malformedSplineTest() throws IOException {
        assertThrows(NoSuchElementException.class,
                () -> TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, GeometryUtil.kRotationZero),
                        List.of(),
                        new Pose2d(0, 0, GeometryUtil.kRotationZero),
                        new TrajectoryConfig(6, 3)));

    }
}
