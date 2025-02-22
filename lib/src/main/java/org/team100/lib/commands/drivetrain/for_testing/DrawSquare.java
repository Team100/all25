package org.team100.lib.commands.drivetrain.for_testing;

import java.util.List;

import org.team100.lib.commands.drivetrain.DriveToPoseWithTrajectory;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.timing.ConstantConstraint;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Drive in a square.
 */
public class DrawSquare extends SequentialCommandGroup implements Glassy {
    private static final double maxVelocityM_S = 2.0;
    private static final double maxAccelM_S_S = 2;

    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final TrajectoryVisualization m_viz;
    private final TrajectoryPlanner m_planner;

    /**
     * Draw a square like so:
     * 
     * .........X
     * ..........
     * .....3---4
     * .....|...|
     * .Y...2---1/5
     */
    public DrawSquare(
            SwerveDriveSubsystem drive,
            SwerveController controller,
            TrajectoryVisualization viz) {
        m_drive = drive;
        m_controller = controller;
        m_viz = viz;
        m_planner = new TrajectoryPlanner(
                List.of(new ConstantConstraint(maxVelocityM_S, maxAccelM_S_S)));

        addCommands(
                go(-0.5, -0.5),
                go(-0.5, 0.5),
                go(0.5, 0.5),
                go(0.5, -0.5),
                go(-0.5, -0.5));
    }

    private Command go(double x, double y) {
        return new DriveToPoseWithTrajectory(
                new Pose2d(x, y, GeometryUtil.kRotationZero),
                m_drive,
                (start, end) -> m_planner.movingToRest(start, end),
                m_controller,
                m_viz);
    }
}
