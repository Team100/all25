package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive from the current state to a field-relative goal.
 * 
 * The trajectory is supplied; the supplier is free to ignore the current state.
 * 
 * The goal rotation is used as the setpoint the entire time, which will put
 * a lot of error into the rotational controller.
 */
public class DriveToWaypoint3 extends Command implements Glassy {
    /**
     * DriveToWaypoint often appears in sequences, the members of which would want
     * to log into the same key space.
     */
    public static class Log {
        private final Pose2dLogger desired;
        private final BooleanLogger aligned;
        private final Pose2dLogger pose;

        public Log(LoggerFactory parent) {
            LoggerFactory log = parent.child("DriveToWaypoint3");
            desired = log.pose2dLogger(Level.TRACE, "Desired");
            aligned = log.booleanLogger(Level.TRACE, "Aligned");
            pose = log.pose2dLogger(Level.TRACE, "Pose");
        }
    }

    private final LoggerFactory m_parent;
    private final Pose2d m_goal;
    private final SwerveDriveSubsystem m_drive;
    private final StraightLineTrajectory m_trajectories;
    private final HolonomicFieldRelativeController m_controller;
    private final TrajectoryVisualization m_viz;
    private final Log m_log;

    private Trajectory100 m_trajectory;

    private ReferenceController m_referenceController;

    /**
     * @param trajectories function that takes a start and end pose and returns a
     *                     trajectory between them.
     */
    public DriveToWaypoint3(
            LoggerFactory parent,
            Log log,
            Pose2d goal,
            SwerveDriveSubsystem drivetrain,
            StraightLineTrajectory trajectories,
            HolonomicFieldRelativeController controller,
            TrajectoryVisualization viz) {
        m_parent = parent;
        m_log = log;
        m_goal = goal;
        m_drive = drivetrain;
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
                new TrajectoryReference(m_trajectory));
        m_viz.setViz(m_trajectory);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
        m_log.aligned.log(() -> m_referenceController.is_aligned());
        m_log.pose.log(() -> m_drive.getState().pose());
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
