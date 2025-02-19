package org.team100.lib.commands.drivetrain;

import java.util.List;

import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.timing.TimingConstraint;
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
    private final List<TimingConstraint> m_constraints;
    private final TrajectoryVisualization m_viz;

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
        m_constraints = new TimingConstraintFactory(swerveKinodynamics).fast();
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Translation2d toGoal = m_goal.getTranslation().minus(m_drive.getPose().getTranslation());
        Pose2d startPose = new Pose2d(m_drive.getPose().getTranslation(), toGoal.getAngle());
        FieldRelativeVelocity startVelocity = m_drive.getVelocity();
        Pose2d startWaypoint = new Pose2d(
                startPose.getTranslation(),
                startVelocity.angle().orElse(toGoal.getAngle()));
        Pose2d endWaypoint = new Pose2d(
                m_goal.getTranslation(),
                m_endVelocity.angle().orElse(toGoal.getAngle()));
        Trajectory100 trajectory = TrajectoryPlanner.generateTrajectory(
                List.of(
                        startWaypoint,
                        endWaypoint),
                List.of(
                        m_drive.getPose().getRotation(),
                        m_goal.getRotation()),
                m_constraints,
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
