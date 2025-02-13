package org.team100.lib.commands.drivetrain;

import java.util.List;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.dashboard.Glassy;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToWaypoint100 extends Command implements Glassy {
    private final Pose2d m_goal;
    private final SwerveDriveSubsystem m_drive;
    private final HolonomicFieldRelativeController m_controller;
    private final List<TimingConstraint> m_constraints;
    private final TrajectoryVisualization m_viz;

    private Trajectory100 m_trajectory = new Trajectory100();

    private ReferenceController m_referenceController;

    public DriveToWaypoint100(
            Pose2d goal,
            SwerveDriveSubsystem drivetrain,
            HolonomicFieldRelativeController controller,
            SwerveKinodynamics swerveKinodynamics,
            double timeBuffer,
            TrajectoryVisualization viz) {
        m_goal = goal;
        m_drive = drivetrain;
        m_controller = controller;
        m_constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        final Pose2d start = m_drive.getPose();
        Pose2d end = m_goal;

        List<Pose2d> waypointsM = getWaypoints(start, end);
        List<Rotation2d> headings = List.of(start.getRotation(), end.getRotation());

        m_trajectory = TrajectoryPlanner.restToRest(
                waypointsM,
                headings,
                m_constraints);

        m_viz.setViz(m_trajectory);

        if (m_trajectory.isEmpty()) {
            m_referenceController = null;
            return;
        }
        m_referenceController = new ReferenceController(
                m_drive,
                m_controller,
                new TrajectoryReference(m_trajectory),
                false);
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

    ////////////////////////////////////////////////////

    /** Waypoints where the rotation points in the direction of motion. */
    private static List<Pose2d> getWaypoints(Pose2d p0, Pose2d p1) {
        Translation2d t0 = p0.getTranslation();
        Translation2d t1 = p1.getTranslation();
        Rotation2d theta = t1.minus(t0).getAngle();
        return List.of(
                new Pose2d(t0, theta),
                new Pose2d(t1, theta));
    }
}
