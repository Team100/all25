package org.team100.lib.motion.arm23;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

class ArmTrajectoriesTest {

    @Test
    void testUnreachable() {
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        ArmTrajectories23 trajectories = new ArmTrajectories23(config);
        Translation2d t0 = new Translation2d(1,1);
        Translation2d t1 = new Translation2d(.6,.6);
        Trajectory trajectory = trajectories.makeTrajectory(t0,t1);
        assertNotNull(trajectory);
    }

    @Test
    void testUnreachable2() {
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        ArmTrajectories23 trajectories = new ArmTrajectories23(config);
        assertNotNull(trajectories);
    }

}
