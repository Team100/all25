package org.team100.lib.commands.arm;

import java.util.Optional;

import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.arm.ArmAngles;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmSubsystem;
import org.team100.lib.motion.arm.ArmTrajectories;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Takt;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Trajectory follower for the two-jointed arm.
 * 
 * Sample a straight-line trajectory in cartesian coordinates, transform
 * position, velocity and acceleration to joint space, and follow with velocity
 * feedforward and positional feedback, both using constant parameters (i.e. no
 * gravity feedforward, no inertia-dependent feedback).
 */
public class ArmTrajectoryCommand extends Command implements Glassy {
    private static final double kTolerance = 0.02;
    private static final TrajectoryConfig kConf = new TrajectoryConfig(0.1, 0.1);
    private static final double kA = 0.2;

    private final ArmSubsystem m_armSubsystem;
    private final ArmKinematics m_armKinematicsM;
    private final Translation2d m_goal;

    private final ArmAngles m_goalAngles;
    private final Takt.Timer m_timer;

    private final Feedback100 m_lowerPosFeedback;
    private final Feedback100 m_upperPosFeedback;
    private final Feedback100 m_lowerVelFeedback;
    private final Feedback100 m_upperVelFeedback;

    private final ArmTrajectories m_trajectories;

    // LOGGERS

    private final DoubleLogger m_log_Lower_FF;
    private final DoubleLogger m_log_Lower_Controller_Output;
    private final DoubleLogger m_log_Upper_FF;
    private final DoubleLogger m_log_Upper_Controller_Output;
    private final DoubleLogger m_log_Lower_Ref;
    private final DoubleLogger m_log_Upper_Ref;
    private final DoubleLogger m_log_Output_Upper;
    private final DoubleLogger m_log_Output_Lower;

    private Trajectory m_trajectory;

    public ArmTrajectoryCommand(
            LoggerFactory parent,
            ArmSubsystem armSubSystem,
            ArmKinematics armKinematicsM,
            Translation2d goal) {
        LoggerFactory child = parent.child(this);
        m_log_Lower_FF = child.doubleLogger(Level.TRACE, "Lower FF");
        m_log_Lower_Controller_Output = child.doubleLogger(Level.TRACE, "Lower Controller Output");
        m_log_Upper_FF = child.doubleLogger(Level.TRACE, "Upper FF");
        m_log_Upper_Controller_Output = child.doubleLogger(Level.TRACE, "Upper Controller Output");
        m_log_Lower_Ref = child.doubleLogger(Level.TRACE, "Lower Ref");
        m_log_Upper_Ref = child.doubleLogger(Level.TRACE, "Upper Ref");
        m_log_Output_Upper = child.doubleLogger(Level.TRACE, "Output Upper");
        m_log_Output_Lower = child.doubleLogger(Level.TRACE, "Output Lower");

        m_armSubsystem = armSubSystem;
        m_armKinematicsM = armKinematicsM;
        m_goal = goal;

        m_goalAngles = m_armKinematicsM.inverse(m_goal);
        m_timer = new Takt.Timer();

        m_lowerPosFeedback = new PIDFeedback(
                child.child("lowerPosController"), 2, 0, 0.1, true, kTolerance, 1);

        m_upperPosFeedback = new PIDFeedback(
                child.child("upperPosController"), 2, 0, 0.05, true, kTolerance, 1);

        m_lowerVelFeedback = new PIDFeedback(
                child.child("lowerVelController"), 0.1, 0, 0, false, kTolerance, 1);

        m_upperVelFeedback = new PIDFeedback(
                child.child("upperVelController"), 0.1, 0, 0, false, kTolerance, 1);

        m_trajectories = new ArmTrajectories(kConf);

        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        Optional<ArmAngles> position = m_armSubsystem.getPosition();
        if (position.isEmpty())
            return;
        m_trajectory = m_trajectories.makeTrajectory(
                m_armKinematicsM.forward(position.get()), m_goal);
    }

    @Override
    public void execute() {
        if (m_trajectory == null)
            return;
        if (m_goalAngles == null)
            return;

        final State desiredState = getDesiredState();

        Optional<ArmAngles> measurement = m_armSubsystem.getPosition();
        if (measurement.isEmpty())
            return;
        Optional<ArmAngles> velocityMeasurement = m_armSubsystem.getVelocity();
        if (velocityMeasurement.isEmpty())
            return;

        // position reference
        final ArmAngles r = getThetaPosReference(desiredState);

        // position feedback
        final double u1_pos = m_lowerPosFeedback.calculate(
                new Model100(measurement.get().th1(), 0),
                new Model100(r.th1(), 0));
        final double u2_pos = m_upperPosFeedback.calculate(
                new Model100(measurement.get().th2(), 0),
                new Model100(r.th2(), 0));

        // velocity reference
        final ArmAngles rdot = getThetaVelReference(desiredState, r);

        // feedforward velocity
        // this is a guess.
        final double kFudgeFactor = 3;
        final double ff2 = rdot.th2() * kFudgeFactor;
        final double ff1 = rdot.th1() * kFudgeFactor;

        // velocity feedback
        final double u1_vel = m_lowerVelFeedback.calculate(
                new Model100(velocityMeasurement.get().th1(), 0),
                new Model100(rdot.th1(), 0));
        final double u2_vel = m_upperVelFeedback.calculate(
                new Model100(velocityMeasurement.get().th2(), 0),
                new Model100(rdot.th2(), 0));

        // u is really velocity
        final double u1 = ff1 + u1_pos + u1_vel;
        final double u2 = ff2 + u2_pos + u2_vel;

        m_armSubsystem.set(u1, u2);

        m_log_Lower_FF.log(() -> ff1);
        m_log_Lower_Controller_Output.log(() -> u1_pos);
        m_log_Upper_FF.log(() -> ff2);
        m_log_Upper_Controller_Output.log(() -> u2_pos);
        m_log_Lower_Ref.log(r::th1);
        m_log_Upper_Ref.log(r::th2);
        m_log_Output_Upper.log(() -> u1);
        m_log_Output_Lower.log(() -> u2);
    }

    private State getDesiredState() {
        double curTime = m_timer.get();
        State state = m_trajectory.sample(curTime);
        // the last state in the trajectory includes whatever the terminal acceleration
        // was, so if you keep sampling past the end, you'll be trying to accelerate
        // away from the endpoint, even though the desired velocity is zero. :-(
        // so we just fix it here:
        if (curTime > m_trajectory.getTotalTimeSeconds())
            state.accelerationMetersPerSecondSq = 0;
        return state;
    }

    ArmAngles getThetaPosReference(State desiredState) {
        Translation2d XYPosReference = desiredState.poseMeters.getTranslation();
        return m_armKinematicsM.inverse(XYPosReference);
    }

    ArmAngles getThetaVelReference(
            State desiredState,
            ArmAngles thetaPosReference) {
        double desiredVecloity = desiredState.velocityMetersPerSecond;
        // accounting for acceleration along the path.
        // in general, we should also account for acceleration across the path, i.e.
        // curvature, but in this case we know the path is a straight line,
        // so just boost the desired velocity a little.
        desiredVecloity += kA * desiredState.accelerationMetersPerSecondSq;
        Rotation2d theta = desiredState.poseMeters.getRotation();
        double desiredXVel = desiredVecloity * theta.getCos();
        double desiredYVel = desiredVecloity * theta.getSin();
        Translation2d XYVelReference = new Translation2d(desiredXVel, desiredYVel);
        return m_armKinematicsM.inverseVel(thetaPosReference, XYVelReference);

    }

    @Override
    public boolean isFinished() {
        if (m_trajectory == null)
            return true;
        if (m_goalAngles == null)
            return true;

        return m_timer.get() > m_trajectory.getTotalTimeSeconds()
                && m_lowerPosFeedback.atSetpoint()
                && m_upperPosFeedback.atSetpoint()
                && m_lowerVelFeedback.atSetpoint()
                && m_upperVelFeedback.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.set(0, 0);
        m_trajectory = null;
    }
}