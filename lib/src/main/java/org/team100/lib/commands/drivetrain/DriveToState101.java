package org.team100.lib.commands.drivetrain;

import java.util.List;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToState101 extends Command implements Glassy {
    private final Pose2d m_goal;
    private final FieldRelativeVelocity m_endVelocity;
    private final SwerveDriveSubsystem m_drive;
    private final HolonomicFieldRelativeController m_controller;
    private final List<TimingConstraint> m_constraints;
    private final TrajectoryVisualization m_viz;

    private ReferenceController m_referenceController;

    public DriveToState101(
            Pose2d goal,
            FieldRelativeVelocity endVelocity,
            SwerveDriveSubsystem drivetrain,
            HolonomicFieldRelativeController controller,
            SwerveKinodynamics swerveKinodynamics,
            TrajectoryVisualization viz) {
        m_goal = goal;
        m_endVelocity = endVelocity;
        m_drive = drivetrain;
        m_controller = controller;
        m_constraints = new TimingConstraintFactory(swerveKinodynamics).fast();
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Translation2d toGoal = m_goal.getTranslation().minus(m_drive.getPose().getTranslation());
        Transform2d transform = new Transform2d(toGoal, toGoal.getAngle()).inverse();
        Pose2d startPose = new Pose2d(m_drive.getPose().getTranslation(), transform.getRotation());
        FieldRelativeVelocity startVelocity = m_drive.getVelocity();
        Pose2d startWaypoint = getStartWaypoint(startPose, startVelocity);
        Pose2d endWaypoint = new Pose2d(m_goal.getTranslation(), new Rotation2d(1, -1));
        List<Pose2d> waypointsM = List.of(
                startWaypoint,
                endWaypoint);
        List<Rotation2d> headings = List.of(
                m_drive.getPose().getRotation(),
                m_goal.getRotation());
        Trajectory100 trajectory = TrajectoryPlanner.generateTrajectory(
                waypointsM,
                headings,
                m_constraints,
                Math.hypot(startVelocity.x(), startVelocity.y()),
                Math.hypot(m_endVelocity.x(), m_endVelocity.y()));

        if (trajectory.length() == 0) {
            m_referenceController = null;
            return;
        }

        m_viz.setViz(trajectory);
        
        m_referenceController = new ReferenceController(m_drive, m_controller, trajectory);
    }

    @Override
    public void execute() {
        if (m_referenceController != null)
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

    private Pose2d getStartWaypoint(Pose2d startPose, FieldRelativeVelocity startVelocity) {
        if (Math.abs(startVelocity.x()) < 0.01 && Math.abs(startVelocity.y()) < 0.01) {
            return startPose;
        } else {
            return new Pose2d(startPose.getTranslation(), new Rotation2d(startVelocity.x(), startVelocity.y()));
        }
    }
}
