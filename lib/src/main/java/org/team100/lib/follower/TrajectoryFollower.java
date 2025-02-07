package org.team100.lib.follower;

import java.util.Optional;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeDeltaLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.logging.LoggerFactory.SwerveModelLogger;
import org.team100.lib.logging.LoggerFactory.TimedPoseLogger;
import org.team100.lib.logging.LoggerFactory.TrajectorySamplePointLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;

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
    private final BooleanLogger m_log_is_mt;
    private final TrajectorySamplePointLogger m_log_sample;

    private final FieldRelativeVelocityLogger m_log_u_FF;
    private final FieldRelativeDeltaLogger m_log_position_error;
    private final FieldRelativeVelocityLogger m_log_u_FB;
    private final FieldRelativeVelocityLogger m_log_velocity_error;
    private final FieldRelativeVelocityLogger m_log_u_VFB;

    private final double m_kPCart;
    private final double m_kPTheta;
    private final double m_kPCartV;
    private final double m_kPThetaV;
    private TrajectoryTimeIterator m_iter;
    private double m_prevTimeS;

    TrajectoryFollower(
            LoggerFactory parent,
            double kPCart,
            double kPTheta,
            double kPCartV,
            double kPThetaV) {
        LoggerFactory log = parent.child("FieldRelativeDrivePIDFFollower");
        m_log_measurement = log.swerveModelLogger(Level.DEBUG, "measurement");
        m_log_setpoint = log.timedPoseLogger(Level.DEBUG, "setpoint");
        m_log_is_mt = log.booleanLogger(Level.TRACE, "IS MT");
        m_log_sample = log.trajectorySamplePointLogger(Level.DEBUG, "sample point");

        m_log_u_FF = log.fieldRelativeVelocityLogger(Level.TRACE, "u_FF");
        m_log_position_error = log.fieldRelativeDeltaLogger(Level.TRACE, "positionError");
        m_log_u_FB = log.fieldRelativeVelocityLogger(Level.TRACE, "u_FB");
        m_log_velocity_error = log.fieldRelativeVelocityLogger(Level.TRACE, "velocityError");
        m_log_u_VFB = log.fieldRelativeVelocityLogger(Level.TRACE, "u_VFB");

        m_kPCart = kPCart;
        m_kPTheta = kPTheta;
        m_kPCartV = kPCartV;
        m_kPThetaV = kPThetaV;
    }

    public void setTrajectory(TrajectoryTimeIterator iter) {
        m_iter = iter;
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
        if (m_iter == null)
            return FieldRelativeVelocity.zero();

        m_log_measurement.log(() -> measurement);

        Optional<TimedPose> optionalSetpoint = getSetpoint(timeS);
        if (!optionalSetpoint.isPresent()) {
            return FieldRelativeVelocity.zero();
        }

        TimedPose setpoint = optionalSetpoint.get();
        m_log_setpoint.log(() -> setpoint);

        FieldRelativeVelocity u_FF = feedforward(setpoint);
        FieldRelativeVelocity u_FB = fullFeedback(measurement, setpoint);
        return u_FF.plus(u_FB);
    }

    public FieldRelativeVelocity positionFeedback(SwerveModel measurement, TimedPose setpoint) {
        FieldRelativeDelta positionError = positionError(measurement.pose(), setpoint);
        FieldRelativeVelocity u_FB = new FieldRelativeVelocity(
                m_kPCart * positionError.getX(),
                m_kPCart * positionError.getY(),
                m_kPTheta * positionError.getRotation().getRadians());
        m_log_u_FB.log(() -> u_FB);
        return u_FB;
    }

    public FieldRelativeVelocity velocityFeedback(SwerveModel currentPose, TimedPose setpoint) {
        final FieldRelativeVelocity velocityError = velocityError(currentPose, setpoint);
        final FieldRelativeVelocity u_VFB = new FieldRelativeVelocity(
                m_kPCartV * velocityError.x(),
                m_kPCartV * velocityError.y(),
                m_kPThetaV * velocityError.theta());
        m_log_u_VFB.log(() -> u_VFB);
        return u_VFB;
    }

    public FieldRelativeVelocity fullFeedback(SwerveModel measurement, TimedPose setpoint) {
        FieldRelativeVelocity u_XFB = positionFeedback(measurement, setpoint);
        FieldRelativeVelocity u_VFB = velocityFeedback(measurement, setpoint);
        return u_XFB.plus(u_VFB);
    }

    /**
     * Note that even though the follower is done, the controller might not be.
     */
    public boolean isDone() {
        return m_iter != null && m_iter.isDone();
    }

    /**
     * this mutates the iterator
     * TODO: fix it.
     */
    Optional<TimedPose> getSetpoint(double timestamp) {
        double mDt = dt(timestamp);
        Optional<TrajectorySamplePoint> sample_point = m_iter.advance(mDt);
        if (!sample_point.isPresent()) {
            m_log_is_mt.log(() -> true);
            return Optional.empty();
        }
        m_log_sample.log(sample_point::get);
        return Optional.of(sample_point.get().state());
    }

    double dt(double timestamp) {
        if (!Double.isFinite(m_prevTimeS))
            m_prevTimeS = timestamp;
        double mDt = timestamp - m_prevTimeS;
        m_prevTimeS = timestamp;
        return mDt;
    }

    public FieldRelativeVelocity feedforward(TimedPose setpoint) {
        SwerveModel model = SwerveModel.fromTimedPose(setpoint);
        m_log_u_FF.log(() -> model.velocity());
        return model.velocity();
    }

    FieldRelativeDelta positionError(Pose2d measurement, TimedPose setpoint) {
        FieldRelativeDelta positionError = FieldRelativeDelta.delta(measurement, setpoint.state().getPose());
        m_log_position_error.log(() -> positionError);
        return positionError;
    }

    FieldRelativeVelocity velocityError(
            SwerveModel measurement,
            TimedPose setpoint) {
        FieldRelativeVelocity velocityError = SwerveModel.fromTimedPose(setpoint).minus(measurement).velocity();
        m_log_velocity_error.log(() -> velocityError);
        return velocityError;
    }
}
