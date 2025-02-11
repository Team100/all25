package org.team100.lib.commands.drivetrain;

import java.util.Iterator;
import java.util.List;
import java.util.function.Function;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.SwerveModelLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follow a list of trajectories with the full state controller.
 */
public class FullStateTrajectoryListCommand extends Command implements Glassy {
    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicFieldRelativeController m_controller;
    private final Function<Pose2d, List<Trajectory100>> m_trajectories;
    private final TrajectoryVisualization m_viz;

    // LOGGERS
    private final SwerveModelLogger m_log_reference;

    private Iterator<Trajectory100> m_trajectoryIter;
    private TrajectoryTimeIterator m_iter;
    private boolean done;
    private boolean m_aligned;

    public FullStateTrajectoryListCommand(
            LoggerFactory parent,
            HolonomicFieldRelativeController controller,
            SwerveDriveSubsystem swerve,
            Function<Pose2d, List<Trajectory100>> trajectories,
            TrajectoryVisualization viz) {
        LoggerFactory child = parent.child(this);
        m_swerve = swerve;
        m_controller = controller;
        m_trajectories = trajectories;
        m_viz = viz;
        addRequirements(m_swerve);
        m_log_reference = child.swerveModelLogger(Level.TRACE, "reference");
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_swerve.getPose();
        m_trajectoryIter = m_trajectories.apply(currentPose).iterator();
        m_iter = null;
        m_aligned = false;
        done = false;
    }

    @Override
    public void execute() {
        if (m_iter == null || m_iter.isDone()) {
            // get the next trajectory
            if (m_trajectoryIter.hasNext()) {
                Trajectory100 m_currentTrajectory = m_trajectoryIter.next();
                m_iter = new TrajectoryTimeIterator(m_currentTrajectory);
                m_viz.setViz(m_currentTrajectory);
                m_aligned = false;
            } else {
                done = true;
                return;
            }
        }

        // now there is a trajectory to follow

        SwerveModel measurement = m_swerve.getState();
        TimedPose curOpt = m_iter.getSample();
        SwerveModel currentReference = SwerveModel.fromTimedPose(curOpt);

        if (m_aligned) {
            TimedPose desiredState = m_iter.advance(TimedRobot100.LOOP_PERIOD_S);

            SwerveModel nextReference = SwerveModel.fromTimedPose(desiredState);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(
                measurement, currentReference, nextReference);
            m_swerve.driveInFieldCoords(fieldRelativeTarget);
            m_log_reference.log(() -> nextReference);
        } else {
            // look one loop ahead by *previewing* the next point
            TimedPose desiredState = m_iter.preview(TimedRobot100.LOOP_PERIOD_S);

            SwerveModel nextReference = SwerveModel.fromTimedPose(desiredState);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(
                measurement, currentReference, nextReference);
            m_aligned = m_swerve.steerAtRest(fieldRelativeTarget);
            m_log_reference.log(() -> nextReference);
        }

    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        m_viz.clear();
    }
}
