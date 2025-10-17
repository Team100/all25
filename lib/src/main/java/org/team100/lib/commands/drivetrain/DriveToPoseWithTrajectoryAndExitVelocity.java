package org.team100.lib.commands.drivetrain;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.reference.TrajectoryReferenceR3;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Drive to a specified pose and exit velocity, using a trajectory constructed
 * at initialization time.
 */
public class DriveToPoseWithTrajectoryAndExitVelocity extends MoveAndHold {
    private final Pose2d m_goal;
    private final GlobalVelocityR3 m_endVelocity;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final TrajectoryVisualization m_viz;
    private final TrajectoryPlanner m_planner;

    private ReferenceController m_referenceController;

    public DriveToPoseWithTrajectoryAndExitVelocity(
            Pose2d goal,
            GlobalVelocityR3 endVelocity,
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
        GlobalVelocityR3 startVelocity = m_drive.getVelocity();
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
                new TrajectoryReferenceR3(trajectory),
                false);
    }

    @Override
    public void execute() {
        if (m_referenceController != null)
            m_referenceController.execute();

    }

    @Override
    public boolean isDone() {
        if (m_referenceController == null)
            return true;
        return m_referenceController.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
    }
}
