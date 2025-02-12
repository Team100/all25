package org.team100.lib.follower;

import java.util.Optional;

import org.team100.lib.controller.drivetrain.FullStateDriveController;
import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.FieldRelativeDeltaLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.logging.LoggerFactory.SwerveModelLogger;
import org.team100.lib.logging.LoggerFactory.TimedPoseLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.Trajectory100;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Follows a trajectory using velocity feedforward, positional feedback, and
 * velocity feedback.
 * 
 * This class uses field-relative coordinates for everything; the transformation
 * to robot-relative should happen downstream.
 */
public class TrajectoryFollower {
    private final SwerveModelLogger m_log_measurement;
    private final TimedPoseLogger m_log_setpoint;
    private final TimedPoseLogger m_log_sample;

    private final FieldRelativeVelocityLogger m_log_u_FF;
    private final FieldRelativeDeltaLogger m_log_position_error;
    private final FieldRelativeVelocityLogger m_log_u_FB;
    private final FieldRelativeVelocityLogger m_log_velocity_error;
    private final FieldRelativeVelocityLogger m_log_u_VFB;

    private final double m_kPCart;
    private final double m_kPTheta;
    private final double m_kPCartV;
    private final double m_kPThetaV;
    private Trajectory100 m_trajectory;
    /** time along the trajectory */
    private double m_timeS;
    private double m_prevTimeS;

    // private final HolonomicFieldRelativeController m_controller;

    public TrajectoryFollower(
            LoggerFactory parent,
            double kPCart,
            double kPTheta,
            double kPCartV,
            double kPThetaV) {
        LoggerFactory log = parent.child("FieldRelativeDrivePIDFFollower");
        m_log_measurement = log.swerveModelLogger(Level.DEBUG, "measurement");
        m_log_setpoint = log.timedPoseLogger(Level.DEBUG, "setpoint");
        m_log_sample = log.timedPoseLogger(Level.DEBUG, "sample point");

        m_log_u_FF = log.fieldRelativeVelocityLogger(Level.TRACE, "u_FF");
        m_log_position_error = log.fieldRelativeDeltaLogger(Level.TRACE, "positionError");
        m_log_u_FB = log.fieldRelativeVelocityLogger(Level.TRACE, "u_FB");
        m_log_velocity_error = log.fieldRelativeVelocityLogger(Level.TRACE, "velocityError");
        m_log_u_VFB = log.fieldRelativeVelocityLogger(Level.TRACE, "u_VFB");

        m_kPCart = kPCart;
        m_kPTheta = kPTheta;
        m_kPCartV = kPCartV;
        m_kPThetaV = kPThetaV;

        // // TODO: pass this in
        // m_controller = FullStateDriveController.getDefault(
        // new HolonomicFieldRelativeController.Log(log));
    }

    public void setTrajectory(Trajectory100 trajectory) {
        m_trajectory = trajectory;
        m_timeS = 0.0;
        m_prevTimeS = Double.POSITIVE_INFINITY;
    }

    /**
     * This uses a single setpoint at the current time to compute the error,
     * which is correct, but it also uses that same setpoint for feed forward, which
     * is not correct. TODO: fix it.
     * 
     * Makes no attempt to enforce feasibility.
     * 
     * TODO: enforce one-call-per-takt.
     * 
     * @param timestamp   in seconds, use Takt.get().
     * @param measurement measured state
     * @return velocity control
     * 
     */
    public FieldRelativeVelocity update(double timeS, SwerveModel measurement) {
        if (m_trajectory == null)
            return FieldRelativeVelocity.zero();
        m_log_measurement.log(() -> measurement);
        TimedPose setpoint = getSetpoint(timeS);
        m_log_setpoint.log(() -> setpoint);
        SwerveModel setpointModel = SwerveModel.fromTimedPose(setpoint);
        FieldRelativeVelocity u_FF = feedforward(setpointModel);
        FieldRelativeVelocity u_FB = fullFeedback(measurement, setpointModel);
        return u_FF.plus(u_FB);
    }

    public FieldRelativeVelocity positionFeedback(SwerveModel measurement, SwerveModel setpointModel) {
        FieldRelativeDelta positionError = positionError(measurement.pose(), setpointModel);
        FieldRelativeVelocity u_FB = new FieldRelativeVelocity(
                m_kPCart * positionError.getX(),
                m_kPCart * positionError.getY(),
                m_kPTheta * positionError.getRotation().getRadians());
        m_log_u_FB.log(() -> u_FB);
        return u_FB;
    }

    public FieldRelativeVelocity velocityFeedback(SwerveModel currentPose, SwerveModel setpointModel) {
        final FieldRelativeVelocity velocityError = velocityError(currentPose, setpointModel);
        final FieldRelativeVelocity u_VFB = new FieldRelativeVelocity(
                m_kPCartV * velocityError.x(),
                m_kPCartV * velocityError.y(),
                m_kPThetaV * velocityError.theta());
        m_log_u_VFB.log(() -> u_VFB);
        return u_VFB;
    }

    public FieldRelativeVelocity fullFeedback(SwerveModel measurement, SwerveModel setpointModel) {
        FieldRelativeVelocity u_XFB = positionFeedback(measurement, setpointModel);
        FieldRelativeVelocity u_VFB = velocityFeedback(measurement, setpointModel);
        return u_XFB.plus(u_VFB);
    }

    /**
     * Note that even though the follower is done, the controller might not be.
     */
    public boolean isDone() {
        return m_trajectory != null && m_trajectory.isDone(m_timeS);
    }

    TimedPose getSetpoint(double timestamp) {
        double mDt = dt(timestamp);
        m_timeS += mDt;
        TimedPose sample_point = m_trajectory.sample(m_timeS);
        m_log_sample.log(() -> sample_point);
        return sample_point;
    }

    double dt(double timestamp) {
        if (!Double.isFinite(m_prevTimeS))
            m_prevTimeS = timestamp;
        double mDt = timestamp - m_prevTimeS;
        m_prevTimeS = timestamp;
        return mDt;
    }

    public FieldRelativeVelocity feedforward(SwerveModel model) {
        m_log_u_FF.log(() -> model.velocity());
        return model.velocity();
    }

    FieldRelativeDelta positionError(Pose2d measurement, SwerveModel setpointModel) {
        FieldRelativeDelta positionError = FieldRelativeDelta.delta(measurement, setpointModel.pose());
        m_log_position_error.log(() -> positionError);
        return positionError;
    }

    FieldRelativeVelocity velocityError(
            SwerveModel measurement,
            SwerveModel setpointModel) {
        FieldRelativeVelocity velocityError = setpointModel.minus(measurement).velocity();
        m_log_velocity_error.log(() -> velocityError);
        return velocityError;
    }
}
