package org.team100.lib.subsystems.se2.commands;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.FullStateControllerSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.se2.MockSubsystemSE2;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TrajectoryFactory;
import org.team100.lib.trajectory.timing.YawRateConstraint;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Demonstrate DriveWithTrajectoryFunction.
 * 
 * https://docs.google.com/spreadsheets/d/1tt7Fq-gkR7aoY6kH2WFVxj4y__SMXiKE0scPG3eHSAk/edit?gid=0#gid=0
 */
public class DriveWithTrajectoryFunctionTest implements Timeless {

    private static final boolean DEBUG = false;

    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    MockSubsystemSE2 subsystem = new MockSubsystemSE2(new ModelSE2());
    FullStateControllerSE2 controller = ControllerFactorySE2.test(log);
    TrajectoryVisualization viz = new TrajectoryVisualization(log);
    List<TimingConstraint> constraints = List.of(
            new ConstantConstraint(log, 2, 2),
            new YawRateConstraint(log, 1, 1));
    PathFactory pathFactory = new PathFactory();
    TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
    TrajectoryPlanner planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);

    /**
     * This is the key to using DriveWithTrajectoryFunction: a function that takes a
     * starting pose and returns a trajectory.
     */
    Trajectory100 makeTrajectory(Pose2d startingPose) {
        return planner.restToRest(
                List.of(
                        WaypointSE2.irrotational(startingPose, 0, 1.2),
                        WaypointSE2.irrotational(
                                new Pose2d(1, 2, new Rotation2d(Math.PI / 2)), Math.PI / 2, 1.2)));
    }

    @Test
    void testDemo() {
        Command drive = new DriveWithTrajectoryFunction(
                log, subsystem, controller, viz, this::makeTrajectory);
        drive.initialize();
        stepTime();
        System.out.println("x, y, theta");
        for (int i = 0; i < 200; ++i) {
            drive.execute();
            subsystem.m_state = new ModelSE2(subsystem.m_state.pose(), subsystem.m_setpoint);
            subsystem.m_state = subsystem.m_state.evolve(0.02);
            Pose2d p = subsystem.m_state.pose();
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f, %6.3f\n", p.getX(), p.getY(), p.getRotation().getRadians());
            stepTime();
        }
    }
}
