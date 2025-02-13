package org.team100.lib.commands.drivetrain;

import java.util.List;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follows a fixed trajectory.
 */
public class FancyTrajectory extends Command implements Glassy {
    private final SwerveDriveSubsystem m_drive;
    private final HolonomicFieldRelativeController m_controller;
    private final List<TimingConstraint> m_constraints;
    private final TrajectoryVisualization m_viz;

    private ReferenceController m_referenceController;

    public FancyTrajectory(
            SwerveDriveSubsystem robotDrive,
            HolonomicFieldRelativeController controller,
            SwerveKinodynamics swerveKinodynamics,
            TrajectoryVisualization viz) {
        m_drive = robotDrive;
        m_controller = controller;
        m_constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        List<Pose2d> waypointsM = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
                new Pose2d(80, 80, Rotation2d.fromDegrees(0)));
        List<Rotation2d> headings = List.of(
                GeometryUtil.fromDegrees(0),
                GeometryUtil.fromDegrees(0));
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_constraints);
        m_referenceController = new ReferenceController(
                m_drive,
                m_controller,
                new TrajectoryReference(trajectory), false);
        m_viz.setViz(trajectory);
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
