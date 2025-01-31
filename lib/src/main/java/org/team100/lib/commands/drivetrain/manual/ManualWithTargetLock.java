package org.team100.lib.commands.drivetrain.manual;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.TargetUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Model100Logger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
 */
public class ManualWithTargetLock implements FieldRelativeDriver {
    private static final double kBallVelocityM_S = 5;
    /**
     * Relative rotational speed. Use a moderate value to trade rotation for
     * translation
     */
    private static final double kRotationSpeed = 0.5;

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Supplier<Translation2d> m_target;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    private final Profile100 m_profile;
    private final BooleanSupplier m_trigger;

    // LOGGERS
    private final DoubleLogger m_log_apparent_motion;
    private final Control100Logger m_log_theta_setpoint;
    private final DoubleLogger m_log_theta_measurement;
    private final DoubleLogger m_log_theta_error;
    private final DoubleLogger m_log_theta_FB;
    private final Model100Logger m_log_omega_reference;
    private final DoubleLogger m_log_omega_measurement;
    private final DoubleLogger m_log_omega_error;
    private final DoubleLogger m_log_omega_FB;
    private final FieldLogger.Log m_field_log;

    private Control100 m_thetaSetpoint;
    private Translation2d m_ball;
    private Translation2d m_ballV;
    private Pose2d m_prevPose;

    public ManualWithTargetLock(
            FieldLogger.Log fieldLogger,
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Translation2d> target,
            PIDController thetaController,
            PIDController omegaController,
            BooleanSupplier trigger) {
        m_field_log = fieldLogger;
        LoggerFactory child = parent.child(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_target = target;
        m_thetaController = thetaController;
        m_omegaController = omegaController;
        m_profile = new TrapezoidProfile100(
                swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed,
                swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed,
                0.01);
        m_trigger = trigger;
        m_log_apparent_motion = child.doubleLogger(Level.TRACE, "apparent motion");
        m_log_theta_setpoint = child.control100Logger(Level.TRACE, "theta/setpoint");
        m_log_theta_measurement = child.doubleLogger(Level.TRACE, "theta/measurement");
        m_log_theta_error = child.doubleLogger(Level.TRACE, "theta/error");
        m_log_theta_FB = child.doubleLogger(Level.TRACE, "theta/fb");
        m_log_omega_reference = child.model100Logger(Level.TRACE, "omega/reference");
        m_log_omega_measurement = child.doubleLogger(Level.TRACE, "omega/measurement");
        m_log_omega_error = child.doubleLogger(Level.TRACE, "omega/error");
        m_log_omega_FB = child.doubleLogger(Level.TRACE, "omega/fb");
    }

    @Override
    public void reset(SwerveModel state) {
        m_thetaSetpoint = state.theta().control();
        m_ball = null;
        m_prevPose = state.pose();
        m_thetaController.reset();
        m_omegaController.reset();
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds, and then desaturates to a feasible holonomic velocity.
     * 
     * @param state from the drivetrain
     * @param input control units [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    @Override
    public FieldRelativeVelocity apply(SwerveModel state, DriverControl.Velocity input) {
        // clip the input to the unit circle
        DriverControl.Velocity clipped = DriveUtil.clampTwist(input, 1.0);
        Rotation2d currentRotation = state.pose().getRotation();
        double headingRate = state.theta().v();

        Translation2d currentTranslation = state.pose().getTranslation();
        Translation2d target = m_target.get();
        Rotation2d bearing = TargetUtil.bearing(currentTranslation, target);

        // take the short path
        double measurement = currentRotation.getRadians();
        bearing = new Rotation2d(
                Math100.getMinDistance(measurement, bearing.getRadians()));

        // make sure the setpoint uses the modulus close to the measurement.
        m_thetaSetpoint = new Control100(
                Math100.getMinDistance(measurement, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());

        // the goal omega should match the target's apparent motion
        double targetMotion = TargetUtil.targetMotion(state, target);
        m_log_apparent_motion.log(() -> targetMotion);

        Model100 goal = new Model100(bearing.getRadians(), targetMotion);

        m_thetaSetpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_thetaSetpoint.model(), goal);

        // this is user input scaled to m/s and rad/s
        FieldRelativeVelocity scaledInput = DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        double thetaFF = m_thetaSetpoint.v();

        double thetaFB = m_thetaController.calculate(measurement, m_thetaSetpoint.x());
        m_log_theta_setpoint.log(() -> m_thetaSetpoint);
        m_log_theta_measurement.log(() -> measurement);
        m_log_theta_error.log(m_thetaController::getPositionError);
        m_log_theta_FB.log(() -> thetaFB);

        double omegaFB = m_omegaController.calculate(headingRate, m_thetaSetpoint.v());
        m_log_omega_reference.log(() -> m_thetaSetpoint.model());
        m_log_omega_measurement.log(() -> headingRate);
        m_log_omega_error.log(m_omegaController::getPositionError);
        m_log_omega_FB.log(() -> omegaFB);

        double omega = MathUtil.clamp(
                thetaFF + thetaFB + omegaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        FieldRelativeVelocity twistWithLockM_S = new FieldRelativeVelocity(scaledInput.x(), scaledInput.y(), omega);

        // desaturate to feasibility by preferring the rotational velocity.
        twistWithLockM_S = m_swerveKinodynamics.preferRotation(twistWithLockM_S);

        m_field_log.m_log_target.log(() -> new double[] {
                target.getX(),
                target.getY(),
                0 });

        // this is just for simulation
        if (m_trigger.getAsBoolean()) {
            m_ball = currentTranslation;
            // correct for newtonian relativity
            m_ballV = new Translation2d(kBallVelocityM_S * TimedRobot100.LOOP_PERIOD_S, currentRotation)
                    .plus(FieldRelativeDelta.delta(m_prevPose, state.pose()).getTranslation());
        }
        if (m_ball != null) {
            m_ball = m_ball.plus(m_ballV);
            m_field_log.m_log_ball.log(() -> new double[] {
                    m_ball.getX(),
                    m_ball.getY(),
                    0 });
        }

        m_prevPose = state.pose();
        return twistWithLockM_S;
    }
}
