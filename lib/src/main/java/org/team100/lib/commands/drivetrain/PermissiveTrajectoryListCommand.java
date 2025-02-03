package org.team100.lib.commands.drivetrain;

import java.util.Iterator;
import java.util.List;
import java.util.Optional;
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
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Similar to TrajectoryListCommand, but each trajectory starts wherever the
 * robot ends up, instead of at the end of the previous trajectory. This is
 * essentially like ignoring cross-track error.
 */
public class PermissiveTrajectoryListCommand extends Command implements Glassy {
    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicFieldRelativeController m_controller;
    private final List<Function<Pose2d, Trajectory100>> m_trajectories;
    private final TrajectoryVisualization m_viz;

    // LOGGERS
    private final SwerveModelLogger m_log_reference;

    private Iterator<Function<Pose2d, Trajectory100>> m_trajectoryIter;
    private Trajectory100 m_currentTrajectory;
    private TrajectoryTimeIterator m_iter;
    private boolean done;
    private boolean m_aligned;

    public PermissiveTrajectoryListCommand(
            LoggerFactory parent,
            SwerveDriveSubsystem swerve,
            HolonomicFieldRelativeController controller,
            List<Function<Pose2d, Trajectory100>> trajectories,
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
        m_controller.reset();
        m_trajectoryIter = m_trajectories.iterator();
        m_currentTrajectory = null;
        done = false;
        m_aligned = false;
    }

    @Override
    public void execute() {
        if (m_currentTrajectory == null || m_iter.isDone()) {
            // get the next trajectory
            if (m_trajectoryIter.hasNext()) {
                m_currentTrajectory = m_trajectoryIter.next().apply(m_swerve.getPose());
                m_iter = new TrajectoryTimeIterator(
                        new TrajectoryTimeSampler(m_currentTrajectory));
                m_viz.setViz(m_currentTrajectory);
                m_aligned = false;
            } else {
                done = true;
                return;
            }
        }

        // now there is a trajectory to follow

        SwerveModel measurement = m_swerve.getState();
        Optional<TrajectorySamplePoint> curOpt = m_iter.getSample();
        if (curOpt.isEmpty()) {
            Util.warn("broken trajectory, cancelling!");
            cancel(); // this should not happen
            return;
        }
        SwerveModel currentReference = SwerveModel.fromTimedPose(curOpt.get().state());

        if (m_aligned) {
            Optional<TrajectorySamplePoint> nextOpt = m_iter.advance(TimedRobot100.LOOP_PERIOD_S);
            if (nextOpt.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TimedPose desiredState = nextOpt.get().state();

            SwerveModel nextReference = SwerveModel.fromTimedPose(desiredState);
            m_log_reference.log(() -> nextReference);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(
                    measurement, currentReference, nextReference);
            m_swerve.driveInFieldCoords(fieldRelativeTarget);
        } else {
            // look just one loop ahead by *previewing* the next point
            Optional<TrajectorySamplePoint> nextOpt = m_iter.preview(TimedRobot100.LOOP_PERIOD_S);
            if (nextOpt.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TimedPose desiredState = nextOpt.get().state();

            SwerveModel nextReference = SwerveModel.fromTimedPose(desiredState);
            m_log_reference.log(() -> nextReference);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(
                    measurement, currentReference, nextReference);
            m_aligned = m_swerve.steerAtRest(fieldRelativeTarget);
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
