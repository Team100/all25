package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryToPose;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive from the current state to a field-relative goal.
 * 
 * The trajectory is supplied; the supplier is free to ignore the current state.
 */
public class DriveToPoseWithTrajectory extends Command implements Glassy {
    private final Pose2d m_goal;
    private final SwerveDriveSubsystem m_drive;
    private final TrajectoryToPose m_trajectories;
    private final SwerveController m_controller;
    private final TrajectoryVisualization m_viz;
 
    private Trajectory100 m_trajectory;

    private ReferenceController m_referenceController;

    /**
     * @param trajectories function that takes a start and end pose and returns a
     *                     trajectory between them.
     */
    public DriveToPoseWithTrajectory(
            Pose2d goal,
            SwerveDriveSubsystem drive,
            TrajectoryToPose trajectories,
            SwerveController controller,
            TrajectoryVisualization viz) {
        m_goal = goal;
        m_drive = drive;
        m_trajectories = trajectories;
        m_controller = controller;
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_trajectory = m_trajectories.apply(m_drive.getState(), m_goal);
        m_referenceController = new ReferenceController(
                m_drive,
                m_controller,
                new TrajectoryReference(m_trajectory),
                false);
        m_viz.setViz(m_trajectory);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public boolean isFinished() {
        return m_referenceController.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
    }
}
