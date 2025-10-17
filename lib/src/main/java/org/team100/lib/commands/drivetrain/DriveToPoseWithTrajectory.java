package org.team100.lib.commands.drivetrain;

import java.util.function.BiFunction;
import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.reference.TrajectoryReferenceR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Drive from the current state to a field-relative goal.
 * 
 * The goal is supplied at initialization time.
 * 
 * The trajectory is supplied by a function that takes both the current pose and
 * the goal. You could use something like `TrajectoryPlanner.movingToRest()` for
 * this function.
 */
public class DriveToPoseWithTrajectory extends MoveAndHold {
    private final Supplier<Pose2d> m_goal;
    private final SwerveDriveSubsystem m_drive;
    private final BiFunction<ModelR3, Pose2d, Trajectory100> m_trajectories;
    private final SwerveController m_controller;
    private final TrajectoryVisualization m_viz;

    private Trajectory100 m_trajectory;

    private ReferenceController m_referenceController;

    /**
     * @param trajectories function that takes a start and end pose and returns a
     *                     trajectory between them.
     */
    public DriveToPoseWithTrajectory(
            Supplier<Pose2d> goal,
            SwerveDriveSubsystem drive,
            BiFunction<ModelR3, Pose2d, Trajectory100> trajectories,
            SwerveController controller,
            TrajectoryVisualization viz) {
        m_goal = goal;
        m_drive = drive;
        m_trajectories = trajectories;
        m_controller = controller;
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_trajectory = m_trajectories.apply(m_drive.getState(), m_goal.get());
        if (m_trajectory.isEmpty()) {
            m_trajectory = null;
            return;
        }
        m_referenceController = new ReferenceController(
                m_drive,
                m_controller,
                new TrajectoryReferenceR3(m_trajectory),
                false);
        m_viz.setViz(m_trajectory);
    }

    @Override
    public void execute() {
        if (m_trajectory == null)
            return;
        m_referenceController.execute();
    }

    @Override
    public boolean isDone() {
        return m_trajectory == null
                || m_referenceController == null
                || m_referenceController.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
        m_referenceController = null;
    }
}
