package org.team100.lib.commands.drivetrain;

import java.util.function.Function;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.reference.TrajectoryReferenceR3;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Follow a trajectory created at initialization time, given the pose at that
 * time. Since the trajectory function takes a pose, and not a state, then
 * probably the returned trajectory should start from rest.
 */
public class DriveWithTrajectoryFunction extends MoveAndHold {
    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final TrajectoryVisualization m_viz;
    private final Function<Pose2d, Trajectory100> m_trajectoryFn;

    /**
     * Non-null when the command is active (between initialize and end), null
     * otherwise.
     */
    private ReferenceController m_referenceController;

    public DriveWithTrajectoryFunction(
            SwerveDriveSubsystem drive,
            SwerveController controller,
            TrajectoryVisualization viz,
            Function<Pose2d, Trajectory100> trajectoryFn) {
        m_drive = drive;
        m_controller = controller;
        m_viz = viz;
        m_trajectoryFn = trajectoryFn;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Trajectory100 trajectory = m_trajectoryFn.apply(m_drive.getPose());
        m_viz.setViz(trajectory);
        m_referenceController = new ReferenceController(
                m_drive,
                m_controller,
                new TrajectoryReferenceR3(trajectory), true);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
        m_referenceController = null;
    }

    @Override
    public boolean isDone() {
        return m_referenceController != null && m_referenceController.isDone();
    }
}
