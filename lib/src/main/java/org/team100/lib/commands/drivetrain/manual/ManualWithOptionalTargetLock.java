package org.team100.lib.commands.drivetrain.manual;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.TargetUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.DriveUtil;
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
    private static final double kRotationSpeed = 0.5;

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Supplier<Optional<Translation2d>> m_target;
    private final Feedback100 m_controller;
    private final Profile100 m_profile;

    private final DoubleLogger m_log_apparent_motion;
    private final FieldLogger.Log m_field_log;

    private Control100 m_thetaSetpoint;

    public ManualWithOptionalTargetLock(
            FieldLogger.Log fieldLogger,
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Optional<Translation2d>> target,
            Feedback100 controller) {
        m_field_log = fieldLogger;
        LoggerFactory child = parent.child(this);
        m_log_apparent_motion = child.doubleLogger(Level.TRACE, "apparent motion");

        m_swerveKinodynamics = swerveKinodynamics;
        m_target = target;
        m_controller = controller;
        m_profile = new TrapezoidProfile100(
                swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed,
                swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed,
                0.01);
    }

    @Override
    public void reset(SwerveModel state) {
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
    public FieldRelativeVelocity apply(
            final SwerveModel state,
            final DriverControl.Velocity input) {

        //
        // feedback is based on the previous setpoint.
        //
        double thetaFB = m_controller.calculate(state.theta(), m_thetaSetpoint.model());

        //
        // feedforward uses the next setpoint
        //

        // clip the input to the unit circle
        DriverControl.Velocity clipped = DriveUtil.clampTwist(input, 1.0);
        Optional<Translation2d> target = m_target.get();
        FieldRelativeVelocity scaledInput = DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        if (!target.isPresent()) {
            // if there's no target, then drive normally.
            return scaledInput;
        }

        final Translation2d currentTranslation = state.pose().getTranslation();
        Rotation2d bearing = TargetUtil.bearing(currentTranslation, target.get()).plus(Rotation2d.kPi);

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

        m_field_log.m_log_target.log(() -> new double[] {
                target.get().getX(),
                target.get().getY(),
                0 });

        return new FieldRelativeVelocity(
                scaledInput.x(),
                scaledInput.y(),
                omega);
    }
}
