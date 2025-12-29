package org.team100.lib.subsystems.swerve.commands.manual;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.controller.r1.Feedback100;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.targeting.TargetUtil;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Manual cartesian control, with rotational control based on a target position.
 * 
 * This is useful for shooting solutions, or for keeping the camera pointed at
 * something.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 * 
 * The targeting solution is based on bearing alone, so it won't work if the
 * robot or target is moving. That effect can be compensated, though.
 * 
 * The difference between this and ManualWithTargetLock is that this can handle
 * an intermittent target.
 */
public class ManualWithOptionalTargetLock implements FieldRelativeDriver {
    /**
     * Relative rotational speed. Use a moderate value to trade rotation for
     * translation
     */
    private static final double ROTATION_SPEED = 0.5;

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Supplier<Optional<Translation2d>> m_target;
    private final Feedback100 m_controller;
    private final IncrementalProfile m_profile;

    private final DoubleLogger m_log_apparent_motion;
    public final DoubleArrayLogger m_log_target;

    private Control100 m_thetaSetpoint;

    public ManualWithOptionalTargetLock(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Optional<Translation2d>> target,
            Feedback100 controller) {
        m_log_target = fieldLogger.doubleArrayLogger(Level.TRACE, "target");
        LoggerFactory log = parent.type(this);
        m_log_apparent_motion = log.doubleLogger(Level.TRACE, "apparent motion");

        m_swerveKinodynamics = swerveKinodynamics;
        m_target = target;
        m_controller = controller;
        m_profile = new TrapezoidIncrementalProfile(
                log,
                swerveKinodynamics.getMaxAngleSpeedRad_S() * ROTATION_SPEED,
                swerveKinodynamics.getMaxAngleAccelRad_S2() * ROTATION_SPEED,
                0.01);
    }

    @Override
    public void reset(ModelSE2 state) {
        m_thetaSetpoint = state.theta().control();
        m_controller.reset();
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds.
     * 
     * @param state from the drivetrain
     * @param input control units [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    @Override
    public VelocitySE2 apply(
            final ModelSE2 state,
            final Velocity input) {

        //
        // feedback is based on the previous setpoint.
        //
        double thetaFB = m_controller.calculate(state.theta(), m_thetaSetpoint.model());

        //
        // feedforward uses the next setpoint
        //

        // clip the input to the unit circle
        Velocity clipped = input.clip(1.0);
        Optional<Translation2d> target = m_target.get();
        VelocitySE2 scaledInput = FieldRelativeDriver.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        if (!target.isPresent()) {
            // if there's no target, then drive normally.
            return scaledInput;
        }

        final Translation2d currentTranslation = state.pose().getTranslation();
        Rotation2d bearing = TargetUtil.absoluteBearing(currentTranslation, target.get()).plus(Rotation2d.kPi);

        final double yaw = state.theta().x();

        // take the short path
        bearing = new Rotation2d(
                Math100.getMinDistance(yaw, bearing.getRadians()));

        // make sure the setpoint uses the modulus close to the measurement.
        m_thetaSetpoint = new Control100(
                Math100.getMinDistance(yaw, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());

        // the goal omega should match the target's apparent motion
        final double targetMotion = TargetUtil.targetMotion(state, target.get());
        m_log_apparent_motion.log(() -> targetMotion);

        Model100 goal = new Model100(bearing.getRadians(), targetMotion);

        m_thetaSetpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_thetaSetpoint, goal);

        // this is user input scaled to m/s and rad/s

        final double thetaFF = m_thetaSetpoint.v();
        final double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        m_log_target.log(() -> new double[] {
                target.get().getX(),
                target.get().getY(),
                0 });

        return new VelocitySE2(
                scaledInput.x(),
                scaledInput.y(),
                omega);
    }
}
