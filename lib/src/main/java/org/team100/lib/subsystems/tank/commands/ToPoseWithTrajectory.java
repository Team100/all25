package org.team100.lib.subsystems.tank.commands;

import java.util.List;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.tank.TankDrive;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TrajectoryFactory;
import org.team100.lib.trajectory.timing.TimedState;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/** Given a goal pose, create a trajectory at initialization and follow it. */
public class ToPoseWithTrajectory extends Command {
    private final Pose2d m_goal;
    private final TankDrive m_drive;
    private final TrajectoryVisualization m_viz;
    private final TrajectoryPlanner m_planner;
    private final LTVUnicycleController m_controller;

    private double m_startTimeS;
    private Trajectory100 m_trajectory;

    public ToPoseWithTrajectory(
            LoggerFactory parent,
            Pose2d goal,
            TankDrive drive,
            TrajectoryVisualization viz) {
        LoggerFactory log = parent.type(this);
        m_goal = goal;
        m_drive = drive;
        m_viz = viz;
        List<TimingConstraint> constraints = List.of(new ConstantConstraint(log, 1, 1));
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(constraints);
        PathFactory pathFactory = new PathFactory();
        m_planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
        m_controller = new LTVUnicycleController(0.020);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_startTimeS = Takt.get();
        // may return null
        m_trajectory = m_planner.restToRest(List.of(
                WaypointSE2.tank(m_drive.getPose()),
                WaypointSE2.tank(m_goal)));
        m_viz.setViz(m_trajectory);
    }

    @Override
    public void execute() {
        try {
            if (m_trajectory == null)
                return;
            // current for position error
            double t = progress();
            TimedState current = m_trajectory.sample(t);
            // next for feedforward (and selecting K)
            TimedState next = m_trajectory.sample(t + TimedRobot100.LOOP_PERIOD_S);
            Pose2d currentPose = m_drive.getPose();
            Pose2d poseReference = current.point().waypoint().pose();
            double velocityReference = next.velocityM_S();
            double omegaReference = next.velocityM_S() * next.point().getHeadingRateRad_M();
            ChassisSpeeds speeds = m_controller.calculate(
                    currentPose, poseReference, velocityReference, omegaReference);
            m_drive.setVelocity(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
        } catch (IllegalStateException e) {
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_viz.clear();
    }

    /** Done when the timer expires. Ignores actual position */
    public boolean isDone() {
        return m_trajectory != null && m_trajectory.isDone(progress());
    }

    /** Time since start */
    private double progress() {
        return Takt.get() - m_startTimeS;
    }

}
