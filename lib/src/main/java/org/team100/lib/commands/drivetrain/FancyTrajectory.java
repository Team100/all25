package org.team100.lib.commands.drivetrain;

import java.util.List;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.follower.TrajectoryFollower;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follows a fixed trajectory.
 */
public class FancyTrajectory extends Command implements Glassy {
    private final SwerveDriveSubsystem m_robotDrive;
    private final TrajectoryFollower m_controller;
    private final List<TimingConstraint> m_constraints;

    // LOGGERS
    private final FieldRelativeVelocityLogger m_log_speed;

    public FancyTrajectory(
            LoggerFactory parent,
            SwerveDriveSubsystem robotDrive,
            TrajectoryFollower controller,
            SwerveKinodynamics swerveKinodynamics) {
        LoggerFactory child = parent.child(this);
        m_log_speed = child.fieldRelativeVelocityLogger(Level.TRACE, "speed");
        m_robotDrive = robotDrive;
        m_controller = controller;
        m_constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        List<Pose2d> waypointsM = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
                new Pose2d(80, 80, Rotation2d.fromDegrees(0)));
        // while turning 180
        List<Rotation2d> headings = List.of(
                GeometryUtil.fromDegrees(0),
                GeometryUtil.fromDegrees(0));

        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_constraints);

        m_controller.setTrajectory(trajectory);
    }

    @Override
    public void execute() {
        FieldRelativeVelocity output = m_controller.update(m_robotDrive.getState());
        m_log_speed.log(() -> output);
        m_robotDrive.driveInFieldCoords(output);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
