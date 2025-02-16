package org.team100.lib.commands.drivetrain.for_testing;

import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.trajectory.TrajectoryToPose;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Choose a position 1m away, go there, then go back, endlessly.
 * It's like DriveToWaypoint3, but with a robot-relative goal set in
 * initialize().
 * 
 * this is for testing odometry.
 */
public class OscillatePosition extends Command implements Glassy {
    private final SwerveDriveSubsystem m_drive;
    private final TrajectoryToPose m_trajectories;
    private final SwerveController m_controller;
    private final double m_offsetM;
    private final TrajectoryVisualization m_viz;

    private Trajectory100 m_trajectory;

    private ReferenceController m_referenceController;

    public OscillatePosition(
            SwerveDriveSubsystem drive,
            TrajectoryToPose trajectories,
            SwerveController controller,
            double offsetM,
            TrajectoryVisualization viz) {
        m_drive = drive;
        m_trajectories = trajectories;
        m_controller = controller;
        m_offsetM = offsetM;
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        // choose a goal 1m away
        SwerveModel start = m_drive.getState();
        Pose2d startPose = start.pose();
        Pose2d endPose = startPose.plus(new Transform2d(m_offsetM, 0, new Rotation2d()));
        m_trajectory = m_trajectories.apply(start, endPose);
        m_referenceController = new ReferenceController(
                m_drive,
                m_controller,
                new TrajectoryReference(m_trajectory),
                true);
        m_viz.setViz(m_trajectory);
    }

    @Override
    public void execute() {
        m_referenceController.execute();

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
