package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follow a single trajectory.
 * 
 * The starting point is fixed, obviously, so this command can
 * only be used from that point. It's kinda just for testing.
 */
public class TrajectoryCommand extends Command implements Glassy {
    private final LoggerFactory m_log;
    private final DriveSubsystemInterface m_swerve;
    private final HolonomicFieldRelativeController m_controller;
    private final Trajectory100 m_trajectory;
    private final TrajectoryVisualization m_viz;
    private ReferenceController m_referenceController;

    public TrajectoryCommand(
            LoggerFactory parent,
            DriveSubsystemInterface swerve,
            HolonomicFieldRelativeController controller,
            Trajectory100 trajectory,
            TrajectoryVisualization viz) {
        m_log = parent.child(this);
        m_swerve = swerve;
        m_controller = controller;
        m_trajectory = trajectory;
        m_viz = viz;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_referenceController = new ReferenceController(m_log, m_swerve, m_controller, m_trajectory);
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

    boolean isDone() {
        return m_referenceController.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        m_viz.clear();
    }

    // for testing
    boolean is_aligned() {
        return m_referenceController.is_aligned();
    }
}
