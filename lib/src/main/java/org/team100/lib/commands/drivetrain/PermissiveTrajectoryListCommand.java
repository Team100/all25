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
    /** progress along trajectory */
    private double m_timeS;

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
        if (m_currentTrajectory == null || m_currentTrajectory.isDone(m_timeS)) {
            // get the next trajectory
            if (m_trajectoryIter.hasNext()) {
                m_currentTrajectory = m_trajectoryIter.next().apply(m_swerve.getPose());
                m_timeS = 0;
                m_viz.setViz(m_currentTrajectory);
                m_aligned = false;
            } else {
                done = true;
                return;
            }
        }

        // now there is a trajectory to follow

        SwerveModel measurement = m_swerve.getState();
        SwerveModel currentReference = SwerveModel.fromTimedPose(m_currentTrajectory.sample(m_timeS));

        if (m_aligned) {
            m_timeS = m_timeS + TimedRobot100.LOOP_PERIOD_S;
            TimedPose desiredState = m_currentTrajectory.sample(m_timeS);

            SwerveModel nextReference = SwerveModel.fromTimedPose(desiredState);
            m_log_reference.log(() -> nextReference);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(
                    measurement, currentReference, nextReference);
            m_swerve.driveInFieldCoords(fieldRelativeTarget);
        } else {
            // look just one loop ahead by *previewing* the next point
            TimedPose desiredState = m_currentTrajectory.sample(m_timeS + TimedRobot100.LOOP_PERIOD_S);

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
