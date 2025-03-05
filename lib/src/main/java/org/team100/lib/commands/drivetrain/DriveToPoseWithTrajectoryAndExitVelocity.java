package org.team100.lib.commands.drivetrain;

import java.util.List;

import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive to a specified pose and exit velocity, using a trajectory constructed
 * at initialization time.
 */
public class DriveToPoseWithTrajectoryAndExitVelocity extends Command implements Glassy {
    private final Pose2d m_goal;
    private final FieldRelativeVelocity m_endVelocity;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final TrajectoryVisualization m_viz;
    private final TrajectoryPlanner m_planner;

    private ReferenceController m_referenceController;

    public DriveToPoseWithTrajectoryAndExitVelocity(
            Pose2d goal,
            FieldRelativeVelocity endVelocity,
            SwerveDriveSubsystem drive,
            SwerveController controller,
            SwerveKinodynamics swerveKinodynamics,
            TrajectoryVisualization viz) {
        m_goal = goal;
        m_endVelocity = endVelocity;
        m_drive = drive;
        m_controller = controller;
        m_planner = new TrajectoryPlanner(new TimingConstraintFactory(swerveKinodynamics).fast());
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Pose2d pose = m_drive.getPose();
        Translation2d toGoal = m_goal.getTranslation().minus(pose.getTranslation());
        FieldRelativeVelocity startVelocity = m_drive.getVelocity();
        HolonomicPose2d startWaypoint = new HolonomicPose2d(
                pose.getTranslation(),
                pose.getRotation(),
                startVelocity.angle().orElse(toGoal.getAngle()));
        HolonomicPose2d endWaypoint = new HolonomicPose2d(
                m_goal.getTranslation(),
                m_goal.getRotation(),
                m_endVelocity.angle().orElse(toGoal.getAngle()));
        Trajectory100 trajectory = m_planner.generateTrajectory(
                List.of(startWaypoint, endWaypoint),
                startVelocity.norm(),
                m_endVelocity.norm());

        if (trajectory.length() == 0) {
            m_referenceController = null;
            return;
        }

        m_viz.setViz(trajectory);

        m_referenceController = new ReferenceController(
                m_drive,
                m_controller,
                new TrajectoryReference(trajectory),
                false);
    }

    @Override
    public void execute() {
        if (m_referenceController != null)
            m_referenceController.execute();

    }

    @Override
    public boolean isFinished() {
        if (m_referenceController == null)
            return true;
        return m_referenceController.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
    }
}
