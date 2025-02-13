package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follows a fixed trajectory.
 */
public class TrajectoryCommand100 extends Command implements Glassy {
    private final SwerveDriveSubsystem m_drive;
    private final Trajectory100 m_trajectory;
    private final HolonomicFieldRelativeController m_controller;
    private final TrajectoryVisualization m_viz;

    private ReferenceController m_referenceController;

    public TrajectoryCommand100(
            SwerveDriveSubsystem robotDrive,
            Trajectory100 trajectory,
            HolonomicFieldRelativeController controller,
            TrajectoryVisualization viz) {
        m_drive = robotDrive;
        m_trajectory = trajectory;
        m_controller = controller;
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_viz.setViz(m_trajectory);
        m_referenceController = new ReferenceController(
                m_drive,
                m_controller,
                new TrajectoryReference(m_trajectory));
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