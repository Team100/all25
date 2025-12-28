package org.team100.lib.subsystems.tank.commands;

import java.util.function.Supplier;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.subsystems.tank.TankDrive;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.timing.TimedState;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follows a trajectory with the WPI LTVUnicycleController, which combines
 * (reference velocity passthrough) feedforward and feedback. The feedback has
 * two independent components:
 * 
 * * translation: robot-relative X (fore/aft) error
 * * rotation: a mix of robot-relative Y (sideways) and rotation errors
 * 
 * The rotation error has a greater effect at greater speeds.
 */
public class FixedTrajectory extends Command {
    // a supplier so that, for example, constraints can be mutable.
    private final Supplier<Trajectory100> m_trajectorySupplier;
    private final TankDrive m_drive;
    private final TrajectoryVisualization m_viz;
    private final LTVUnicycleController m_controller;
    private double m_startTimeS;
    private Trajectory100 m_trajectory;

    public FixedTrajectory(
            Supplier<Trajectory100> trajectorySupplier,
            TankDrive drive,
            TrajectoryVisualization viz) {
        m_trajectorySupplier = trajectorySupplier;
        m_drive = drive;
        m_viz = viz;
        m_controller = new LTVUnicycleController(0.020);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_startTimeS = Takt.get();
        m_trajectory = m_trajectorySupplier.get();
        if (m_trajectory == null)
            return;
        m_viz.setViz(m_trajectory);
    }

    @Override
    public void execute() {
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
