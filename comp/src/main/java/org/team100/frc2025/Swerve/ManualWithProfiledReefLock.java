package org.team100.frc2025.Swerve;

import java.util.function.Supplier;

import org.team100.lib.controller.r1.Feedback100;
import org.team100.lib.field.FieldConstants;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.commands.manual.FieldRelativeDriver;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Function that supports manual cartesian control, and both manual and locked
 * rotational control.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 * 
 * The profile depends on robot speed, making rotation the lowest priority.
 */
public class ManualWithProfiledReefLock implements FieldRelativeDriver {
    // don't try to go full speed
    private static final double PROFILE_SPEED = 0.6;
    // accelerate gently to avoid upset
    private static final double PROFILE_ACCEL = 0.5;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Supplier<Translation2d> m_robotLocation;

    /** lock rotation to reef center */
    private final Supplier<Boolean> m_lockToReef;

    private final Feedback100 m_thetaFeedback;

    // LOGGERS
    private final BooleanLogger m_log_snap_mode;
    private final DoubleLogger m_log_max_speed;
    private final DoubleLogger m_log_max_accel;
    private final DoubleLogger m_log_goal_theta;
    private final Control100Logger m_log_setpoint_theta;
    private final DoubleLogger m_log_theta_FF;
    private final DoubleLogger m_log_theta_FB;
    private final DoubleLogger m_log_output_omega;
    private final LoggerFactory m_log;

    // package private for testing

    Control100 m_thetaSetpoint = null;

    public ManualWithProfiledReefLock(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Boolean> lockToReef,
            Feedback100 thetaController,
            Supplier<Translation2d> robotLocation) {
        m_log = parent.type(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_lockToReef = lockToReef;
        m_robotLocation = robotLocation;
        m_thetaFeedback = thetaController;
        m_log_snap_mode = m_log.booleanLogger(Level.TRACE, "snap mode");
        m_log_max_speed = m_log.doubleLogger(Level.TRACE, "maxSpeedRad_S");
        m_log_max_accel = m_log.doubleLogger(Level.TRACE, "maxAccelRad_S2");
        m_log_goal_theta = m_log.doubleLogger(Level.TRACE, "goal/theta");
        m_log_setpoint_theta = m_log.control100Logger(Level.TRACE, "setpoint/theta");
        m_log_theta_FF = m_log.doubleLogger(Level.TRACE, "thetaFF");
        m_log_theta_FB = m_log.doubleLogger(Level.TRACE, "thetaFB");
        m_log_output_omega = m_log.doubleLogger(Level.TRACE, "output/omega");
    }

    @Override
    public void reset(ModelSE2 state) {
        m_thetaSetpoint = state.theta().control();
        m_thetaFeedback.reset();
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds.
     * 
     * If you touch the POV and not the twist rotation, it remembers the POV. if you
     * use the twist rotation, it forgets and just uses that.
     * 
     * Desaturation prefers the rotational profile completely in the snap case, and
     * normally in the non-snap case.
     * 
     * @param state    current drivetrain state from the pose estimator
     * @param twist1_1 control units, [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    @Override
    public VelocitySE2 apply(
            final ModelSE2 state,
            final Velocity twist1_1) {
        final VelocitySE2 control = clipAndScale(twist1_1);

        if (!m_lockToReef.get()) {
            // not locked, just return the input.
            m_thetaSetpoint = null;
            m_log_snap_mode.log(() -> false);
            return control;
        }

        // if this is the first run since the latch, then the setpoint should be
        // whatever the measurement is
        if (m_thetaSetpoint == null) {
            m_thetaSetpoint = state.theta().control();
        }

        // feedback uses the current setpoint, which was set previously
        final double thetaFB = m_thetaFeedback.calculate(state.theta(), m_thetaSetpoint.model());

        final double yawMeasurement = state.theta().x();
        // take the short path
        Rotation2d m_goal = Math100.getMinDistance(
                yawMeasurement,
                FieldConstants.angleToReefCenter(m_robotLocation.get()));

        // use the modulus closest to the measurement
        m_thetaSetpoint = new Control100(
                Math100.getMinDistance(yawMeasurement, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());

        final TrapezoidIncrementalProfile profile = makeProfile(state.velocity().norm());
        m_thetaSetpoint = profile.calculate(
                TimedRobot100.LOOP_PERIOD_S, m_thetaSetpoint, new Model100(m_goal.getRadians(), 0));

        final double thetaFF = m_thetaSetpoint.v();

        final double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        VelocitySE2 twistWithSnapM_S = new VelocitySE2(control.x(), control.y(), omega);

        m_log_snap_mode.log(() -> true);
        m_log_goal_theta.log(m_goal::getRadians);
        m_log_setpoint_theta.log(() -> m_thetaSetpoint);
        m_log_theta_FF.log(() -> thetaFF);
        m_log_theta_FB.log(() -> thetaFB);
        m_log_output_omega.log(() -> omega);

        return twistWithSnapM_S;
    }

    public VelocitySE2 clipAndScale(Velocity twist1_1) {
        // clip the input to the unit circle
        final Velocity clipped = twist1_1.clip(1.0);

        // scale to max in both translation and rotation
        return FieldRelativeDriver.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
    }

    /**
     * Note that the max speed and accel are inversely proportional to the current
     * velocity.
     * 
     * TODO: get rid of this
     */
    public TrapezoidIncrementalProfile makeProfile(double currentVelocity) {
        // fraction of the maximum speed
        final double xyRatio = Math.min(1, currentVelocity / m_swerveKinodynamics.getMaxDriveVelocityM_S());
        // fraction left for rotation
        final double oRatio = 1 - xyRatio;
        // add a little bit of default speed
        final double rotationSpeed = Math.max(0.1, oRatio);

        final double maxSpeedRad_S = m_swerveKinodynamics.getMaxAngleSpeedRad_S() * rotationSpeed * PROFILE_SPEED;

        final double maxAccelRad_S2 = m_swerveKinodynamics.getMaxAngleAccelRad_S2() * rotationSpeed * PROFILE_ACCEL;

        m_log_max_speed.log(() -> maxSpeedRad_S);
        m_log_max_accel.log(() -> maxAccelRad_S2);

        return new TrapezoidIncrementalProfile(
                m_log,
                maxSpeedRad_S,
                maxAccelRad_S2,
                0.01);
    }
}
