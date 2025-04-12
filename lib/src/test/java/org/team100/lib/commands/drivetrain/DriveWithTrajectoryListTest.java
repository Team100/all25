package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.state.Control100;
import org.team100.lib.testing.Timeless;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj.DataLogManager;

class DriveWithTrajectoryListTest extends Fixtured implements Timeless {
    private static final boolean DEBUG = false;
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TrajectoryVisualization viz = new TrajectoryVisualization(logger);

    List<TimingConstraint> constraints = new TimingConstraintFactory(fixture.swerveKinodynamics).allGood();
    TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

    @BeforeEach
    void nolog() {
        DataLogManager.stop();
    }

    @Test
    void testSimple() {
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);
        SwerveController control = SwerveControllerFactory.test(logger);
        DriveWithTrajectoryList c = new DriveWithTrajectoryList(
                fixture.drive,
                control,
                x -> List.of(planner.line(x)),
                viz);
        c.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), kDelta);
        c.execute();
        assertFalse(c.isFinished());
        // the trajectory takes a little over 3s
        for (double t = 0; t < 3.1; t += TimedRobot100.LOOP_PERIOD_S) {
            stepTime();
            c.execute();
            fixture.drive.periodic(); // for updateOdometry
        }
        // at goal; wide tolerance due to test timing
        assertTrue(c.isFinished());
        assertEquals(1.031, fixture.drive.getPose().getX(), 0.05);
    }
}
