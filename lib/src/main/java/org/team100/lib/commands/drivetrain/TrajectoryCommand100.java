package org.team100.lib.commands.drivetrain;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.follower.TrajectoryFollower;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.util.Takt;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follows a fixed trajectory.
 */
public class TrajectoryCommand100 extends Command implements Glassy  {
    /**
     * Log exists so multiple commands can use the same keys.
     */
    public static class Log {
        private final Pose2dLogger m_log_goal;
        private final FieldRelativeVelocityLogger m_log_speed;
        private final DoubleLogger m_log_THETA_ERROR;
        private final BooleanLogger m_log_FINSIHED;

        public Log(LoggerFactory parent) {
            LoggerFactory log = parent.child("TrajectoryCommand100");
            m_log_goal = log.pose2dLogger(Level.TRACE, "goal");
            m_log_speed = log.fieldRelativeVelocityLogger(Level.TRACE, "speed");
            m_log_THETA_ERROR = log.doubleLogger(Level.TRACE, "THETA ERROR");
            m_log_FINSIHED = log.booleanLogger(Level.TRACE, "FINSIHED");
        }
    }

    private final Log m_log;
    private final SwerveDriveSubsystem m_robotDrive;
    private final TrajectoryFollower m_controller;
    private final Trajectory100 m_trajectory;
    private final Pose2d m_goal;
    private final TrajectoryVisualization m_viz;

    public TrajectoryCommand100(
            Log log,
            SwerveDriveSubsystem robotDrive,
            Trajectory100 trajectory,
            TrajectoryFollower controller,
            TrajectoryVisualization viz) {
        m_log = log;
        m_robotDrive = robotDrive;
        m_trajectory = trajectory;
        m_controller = controller;
        m_goal = m_trajectory.getLastPoint().state().getPose();
        m_viz = viz;
        log.m_log_goal.log(() -> m_goal);
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        m_viz.setViz(m_trajectory);
        m_controller.setTrajectory(m_trajectory);
    }

    @Override
    public void execute() {
        final double now = Takt.get();
        FieldRelativeVelocity output = m_controller.update(now, m_robotDrive.getState());

        m_robotDrive.driveInFieldCoords(output);

        m_log.m_log_speed.log(() -> output);
        double thetaErrorRad = m_goal.getRotation().getRadians()
                - m_robotDrive.getPose().getRotation().getRadians();
        m_log.m_log_THETA_ERROR.log(() -> thetaErrorRad);
        m_log.m_log_FINSIHED.log(() -> false);
    }

    @Override
    public boolean isFinished() {
        return m_controller.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        m_log.m_log_FINSIHED.log(() -> true);
        m_robotDrive.stop();
        m_viz.clear();
    }
}