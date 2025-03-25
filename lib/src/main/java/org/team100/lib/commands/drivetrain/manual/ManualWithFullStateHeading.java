package org.team100.lib.commands.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.HeadingLatch;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Function that supports manual cartesian control, and both manual and locked
 * rotational control.
 * 
 * Rotation uses simple full-state feedback and that's all..
 */
public class ManualWithFullStateHeading implements FieldRelativeDriver {
    /**
     * in "gentle snaps" mode this is the max omega allowed. The
     * idea is to get to the setpoint over a few seconds, so plan ahead!
     */
    private static final double GENTLE_OMEGA = Math.PI / 2;
    private final SwerveKinodynamics m_swerveKinodynamics;
    /** Absolute input supplier, null if free */
    private final Supplier<Rotation2d> m_desiredRotation;
    private final HeadingLatch m_latch;
    // feedback gains
    private final double[] m_K;
    private final LinearFilter m_outputFilter;

    private final BooleanLogger m_log_snap_mode;
    private final DoubleLogger m_log_goal_theta;
    private final Control100Logger m_log_setpoint_theta;
    private final DoubleLogger m_log_measurement_theta;
    private final DoubleLogger m_log_measurement_omega;
    private final DoubleLogger m_log_error_theta;
    private final DoubleLogger m_log_error_omega;
    private final DoubleLogger m_log_theta_FB;
    private final DoubleLogger m_log_omega_FB;
    private final DoubleLogger m_log_output_omega;

    // package private for testing
    Rotation2d m_goal = null;
    Control100 m_thetaSetpoint = null;

    /**
     * 
     * @param parent
     * @param swerveKinodynamics
     * @param desiredRotation    absolute input supplier, null if free. usually
     *                           POV-derived.
     * @param k                  full state gains
     */
    public ManualWithFullStateHeading(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Rotation2d> desiredRotation,
            double[] k) {
        LoggerFactory child = parent.child(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_desiredRotation = desiredRotation;
        m_K = k;
        m_latch = new HeadingLatch();
        m_outputFilter = LinearFilter.singlePoleIIR(0.01, TimedRobot100.LOOP_PERIOD_S);
        m_log_snap_mode = child.booleanLogger(Level.TRACE, "snap mode");
        m_log_goal_theta = child.doubleLogger(Level.DEBUG, "goal/theta");
        m_log_setpoint_theta = child.control100Logger(Level.DEBUG, "setpoint/theta");
        m_log_measurement_theta = child.doubleLogger(Level.DEBUG, "measurement/theta");
        m_log_measurement_omega = child.doubleLogger(Level.DEBUG, "measurement/omega");
        m_log_error_theta = child.doubleLogger(Level.TRACE, "error/theta");
        m_log_error_omega = child.doubleLogger(Level.TRACE, "error/omega");
        m_log_theta_FB = child.doubleLogger(Level.TRACE, "thetaFB");
        m_log_omega_FB = child.doubleLogger(Level.TRACE, "omegaFB");
        m_log_output_omega = child.doubleLogger(Level.TRACE, "output/omega");
    }

    @Override
    public void reset(SwerveModel state) {
        m_thetaSetpoint = state.theta().control();
        m_goal = null;
        m_latch.unlatch();
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
    public FieldRelativeVelocity apply(
            final SwerveModel state,
            final DriverControl.Velocity twist1_1) {
        final Model100 thetaState = state.theta();
        final double yawMeasurement = thetaState.x();
        final double yawRate = thetaState.v();

        // clip the input to the unit circle
        final DriverControl.Velocity clipped = DriveUtil.clampTwist(twist1_1, 1.0);
        // scale to max in both translation and rotation
        final FieldRelativeVelocity scaled = DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        final Rotation2d pov = m_desiredRotation.get();
        m_goal = m_latch.latchedRotation(
                m_swerveKinodynamics.getMaxAngleAccelRad_S2(),
                state.theta(),
                pov,
                scaled.theta());
        if (m_goal == null) {
            // we're not in snap mode, so it's pure manual
            // in this case there is no setpoint
            m_thetaSetpoint = null;
            m_log_snap_mode.log(() -> false);
            return scaled;
        }

        // take the short path
        m_goal = new Rotation2d(
                Math100.getMinDistance(yawMeasurement, m_goal.getRadians()));

        // in snap mode we take dx and dy from the user, and control dtheta.
        // the omega goal in snap mode is always zero.
        m_thetaSetpoint = new Control100(m_goal.getRadians(), 0);

        final double thetaError = MathUtil.angleModulus(m_thetaSetpoint.x() - yawMeasurement);
        final double omegaError = -1.0 * yawRate;

        final double omegaFB = getOmegaFB(omegaError);
        final double thetaFB = getThetaFB(thetaError);
        double totalFB = thetaFB + omegaFB;
        if (Experiments.instance.enabled(Experiment.SnapGentle))
            totalFB = MathUtil.clamp(totalFB, -1.0 * GENTLE_OMEGA, GENTLE_OMEGA);

        final double omega = MathUtil.clamp(
                totalFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        final FieldRelativeVelocity withSnap = new FieldRelativeVelocity(scaled.x(), scaled.y(), omega);

        m_log_snap_mode.log(() -> true);
        m_log_goal_theta.log(m_goal::getRadians);
        m_log_setpoint_theta.log(() -> m_thetaSetpoint);
        m_log_measurement_theta.log(() -> yawMeasurement);
        m_log_measurement_omega.log(() -> yawRate);
        m_log_error_theta.log(() -> thetaError);
        m_log_error_omega.log(() -> omegaError);
        m_log_theta_FB.log(() -> thetaFB);
        m_log_omega_FB.log(() -> omegaFB);
        m_log_output_omega.log(() -> omega);

        return withSnap;
    }

    private double getOmegaFB(final double omegaError) {
        final double omegaFB = m_K[1] * omegaError;

        if (Experiments.instance.enabled(Experiment.SnapThetaFilter)) {
            // output filtering to prevent oscillation due to delay
            return m_outputFilter.calculate(omegaFB);
        }
        if (Math.abs(omegaFB) < 0.05) {
            return 0;
        }
        return omegaFB;
    }

    private double getThetaFB(final double thetaError) {
        final double thetaFB = m_K[0] * thetaError;
        if (Math.abs(thetaFB) < 0.05) {
            return 0;
        }
        return thetaFB;
    }
}
