package org.team100.lib.commands.drivetrain;

import java.util.Iterator;
import java.util.List;
import java.util.function.Function;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follow a list of trajectories.
 * 
 * The list can be relative to the current pose.
 */
public class TrajectoryListCommand extends Command implements Glassy {
    private final LoggerFactory m_log;
    private final SwerveDriveSubsystem m_drive;
    private final HolonomicFieldRelativeController m_controller;
    private final Function<Pose2d, List<Trajectory100>> m_trajectories;
    private final TrajectoryVisualization m_viz;

    private Iterator<Trajectory100> m_trajectoryIter;
    private ReferenceController m_referenceController;

    public TrajectoryListCommand(
            LoggerFactory parent,
            SwerveDriveSubsystem swerve,
            HolonomicFieldRelativeController controller,
            Function<Pose2d, List<Trajectory100>> trajectories,
            TrajectoryVisualization viz) {
        m_log = parent.child(this);
        m_drive = swerve;
        m_controller = controller;
        m_trajectories = trajectories;
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_trajectoryIter = m_trajectories.apply(m_drive.getPose()).iterator();
        m_referenceController = null;
    }

    @Override
    public void execute() {
        if (m_referenceController == null || m_referenceController.isFinished()) {
            // get the next trajectory
            if (m_trajectoryIter.hasNext()) {
                Trajectory100 m_trajectory = m_trajectoryIter.next();
                m_referenceController = new ReferenceController(m_drive, m_controller, m_trajectory);
                m_viz.setViz(m_trajectory);
            } else {
                return;
            }
        }

        // now there is a trajectory to follow
        if (m_referenceController != null)
            m_referenceController.execute();

    }

    @Override
    public boolean isFinished() {
        return m_referenceController == null || m_referenceController.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
    }
}
