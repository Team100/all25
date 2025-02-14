package org.team100.lib.commands.drivetrain;

import java.util.Iterator;
import java.util.List;
import java.util.function.Function;

import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Similar to TrajectoryListCommand, but each trajectory starts wherever the
 * robot ends up, instead of at the end of the previous trajectory. This is
 * essentially like ignoring cross-track error.
 */
public class PermissiveTrajectoryListCommand extends Command implements Glassy {
    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final List<Function<Pose2d, Trajectory100>> m_trajectories;
    private final TrajectoryVisualization m_viz;

    private Iterator<Function<Pose2d, Trajectory100>> m_trajectoryIter;
    private ReferenceController m_referenceController;

    public PermissiveTrajectoryListCommand(
            SwerveDriveSubsystem swerve,
            SwerveController controller,
            List<Function<Pose2d, Trajectory100>> trajectories,
            TrajectoryVisualization viz) {
        m_drive = swerve;
        m_controller = controller;
        m_trajectories = trajectories;
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_trajectoryIter = m_trajectories.iterator();
        m_referenceController = null;
    }

    @Override
    public void execute() {
        if (m_referenceController == null || m_referenceController.isFinished()) {
            // get the next trajectory
            if (m_trajectoryIter.hasNext()) {
                Trajectory100 trajectory = m_trajectoryIter.next().apply(m_drive.getPose());
                m_referenceController = new ReferenceController(
                        m_drive,
                        m_controller,
                        new TrajectoryReference(trajectory),
                        false);
                m_viz.setViz(trajectory);
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
