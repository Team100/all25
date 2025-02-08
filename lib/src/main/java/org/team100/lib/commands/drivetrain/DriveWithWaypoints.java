package org.team100.lib.commands.drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.follower.TrajectoryFollower;
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
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.util.Takt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveWithWaypoints extends Command implements Glassy {
    private final SwerveDriveSubsystem m_swerve;
    private final TrajectoryFollower m_controller;
    private final List<TimingConstraint> constraints;
    private final Supplier<List<Pose2d>> m_goal;

    // LOGGERS
    private final FieldRelativeVelocityLogger m_log_speed;

    public DriveWithWaypoints(
            LoggerFactory parent,
            SwerveDriveSubsystem drivetrain,
            TrajectoryFollower controller,
            SwerveKinodynamics limits,
            Supplier<List<Pose2d>> goal) {
        LoggerFactory child = parent.child(this);
        m_log_speed = child.fieldRelativeVelocityLogger(Level.TRACE, "speed");
        m_swerve = drivetrain;
        m_controller = controller;
        constraints = new TimingConstraintFactory(limits).fast();
        m_goal = goal;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        final Pose2d start = m_swerve.getPose();
        List<Pose2d> newWaypointM = new ArrayList<>(m_goal.get());
        newWaypointM.add(0, start);

        List<Rotation2d> headings = new ArrayList<>();
        for (int i = 0; i < newWaypointM.size(); i++) {
            headings.add(newWaypointM.get(i).getRotation());
        }

        newWaypointM = getWaypointsList(newWaypointM);

        Trajectory100 trajectory = TrajectoryPlanner.restToRest(newWaypointM, headings, constraints);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(trajectory));
        m_controller.setTrajectory(iter);
    }

    @Override
    public void execute() {
        double now = Takt.get();
        FieldRelativeVelocity output = m_controller.update(now, m_swerve.getState());
        m_log_speed.log(() -> output);
        m_swerve.driveInFieldCoords(output);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return m_controller.isDone();
    }

    private static List<Pose2d> getWaypointsList(List<Pose2d> m) {
        List<Pose2d> waypointsM = new ArrayList<>();
        for (int i = 0; i < m.size() - 1; i += 1) {
            Translation2d t0 = m.get(i).getTranslation();
            Translation2d t1 = m.get(i + 1).getTranslation();
            Rotation2d theta = t1.minus(t0).getAngle();
            waypointsM.add(new Pose2d(t0, theta));
        }

        Translation2d t0 = m.get(m.size() - 1).getTranslation();
        Translation2d t1 = m.get(m.size() - 2).getTranslation();
        Rotation2d theta = t0.minus(t1).getAngle();
        waypointsM.add(new Pose2d(t0, theta));
        return waypointsM;
    }
}
