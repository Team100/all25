package org.team100.lib.commands.drivetrain;

import java.util.Optional;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.SwerveModelLogger;
import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follow a single trajectory.
 * 
 * The starting point is fixed, obviously, so this command can
 * only be used from that point. It's kinda just for testing.
 */
public class TrajectoryCommand extends Command implements Glassy {
    private final DriveSubsystemInterface m_swerve;
    private final HolonomicFieldRelativeController m_controller;
    private final TrajectoryVisualization m_viz;
    // LOGGERS
    private final SwerveModelLogger m_log_reference;

    private Trajectory100 m_trajectory;
    private TrajectoryTimeIterator m_iter;
    boolean m_aligned;
    private boolean done;

    public TrajectoryCommand(
            LoggerFactory parent,
            DriveSubsystemInterface swerve,
            HolonomicFieldRelativeController controller,
            Trajectory100 trajectory,
            TrajectoryVisualization viz) {
        LoggerFactory child = parent.child(this);
        m_swerve = swerve;
        m_controller = controller;
        m_trajectory = trajectory;
        m_viz = viz;
        addRequirements(m_swerve);
        m_log_reference = child.swerveModelLogger(Level.TRACE, "reference");
    }

    @Override
    public void initialize() {
        m_controller.reset();
        m_iter = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(m_trajectory));
        m_viz.setViz(m_trajectory);
        done = false;
        m_aligned = false;
    }

    @Override
    public void execute() {
        Util.println("********** EXECUTE");
        if (m_iter.isDone()) {
            done = true;
            return;
        }

        // now there is a trajectory to follow

        SwerveModel measurement = m_swerve.getState();
        Util.printf("TrajectoryCommand measurement %s\n", measurement);
        Optional<TrajectorySamplePoint> curOpt = m_iter.getSample();
        if (curOpt.isEmpty()) {
            Util.warn("broken trajectory, cancelling!");
            cancel(); // this should not happen
            return;
        }
        TimedPose state = curOpt.get().state();
        if (state.velocityM_S() > 0) {
            // if we're moving, don't worry about the steering.
            // this catches the "start from rest" case and allows
            // "start from moving" to work without interference.
            m_aligned = true;
        }

        SwerveModel currentReference = SwerveModel.fromTimedPose(state);

        if (m_aligned) {
            Optional<TrajectorySamplePoint> nextOpt = m_iter.advance(TimedRobot100.LOOP_PERIOD_S);
            if (nextOpt.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TimedPose desiredState = nextOpt.get().state();
            Util.printf("advance %s\n", desiredState);

            SwerveModel nextReference = SwerveModel.fromTimedPose(desiredState);
            m_log_reference.log(() -> nextReference);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(
                    measurement, currentReference, nextReference);
            m_swerve.driveInFieldCoords(fieldRelativeTarget);
        } else {
            // look one loop ahead by *previewing* the next point
            Optional<TrajectorySamplePoint> nextOpt = m_iter.preview(TimedRobot100.LOOP_PERIOD_S);
            if (nextOpt.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TimedPose desiredState = nextOpt.get().state();
            Util.printf("preview %s\n", desiredState);

            SwerveModel nextReference = SwerveModel.fromTimedPose(desiredState);
            m_log_reference.log(() -> nextReference);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(
                    measurement, currentReference, nextReference);
            Util.printf("target %s\n", fieldRelativeTarget);
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
