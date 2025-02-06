package org.team100.lib.follower;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.FieldRelativeDeltaLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;

/** Stateless (except for loggers) calculations for trajectory followers. */
public class DriveTrajectoryFollowerUtil implements Glassy {
    // LOGGERS
    private final FieldRelativeVelocityLogger m_log_u_FF;
    private final FieldRelativeDeltaLogger m_log_position_error;
    private final FieldRelativeVelocityLogger m_log_u_FB;
    private final FieldRelativeVelocityLogger m_log_velocity_error;
    private final FieldRelativeVelocityLogger m_log_u_VFB;

    public DriveTrajectoryFollowerUtil(LoggerFactory parent) {
        LoggerFactory child = parent.child(this);
        m_log_u_FF = child.fieldRelativeVelocityLogger(Level.TRACE, "u_FF");
        m_log_position_error = child.fieldRelativeDeltaLogger(Level.TRACE, "positionError");
        m_log_u_FB = child.fieldRelativeVelocityLogger(Level.TRACE, "u_FB");
        m_log_velocity_error = child.fieldRelativeVelocityLogger(Level.TRACE, "velocityError");
        m_log_u_VFB = child.fieldRelativeVelocityLogger(Level.TRACE, "u_VFB");
    }

    public FieldRelativeVelocity fieldRelativeFeedforward(TimedPose setpoint) {
        SwerveModel model = SwerveModel.fromTimedPose(setpoint);
        m_log_u_FF.log(() -> model.velocity());
        return model.velocity();
    }

    public FieldRelativeVelocity fieldRelativeFeedback(
            SwerveModel measurement,
            TimedPose setpoint,
            double kPCart,
            double kPTheta) {
        FieldRelativeDelta positionError = fieldRelativeError(measurement.pose(), setpoint);
        m_log_position_error.log(() -> positionError);
        FieldRelativeVelocity u_FB = new FieldRelativeVelocity(
                kPCart * positionError.getX(),
                kPCart * positionError.getY(),
                kPTheta * positionError.getRotation().getRadians());
        m_log_u_FB.log(() -> u_FB);
        return u_FB;
    }

    public FieldRelativeVelocity fieldRelativeVelocityFeedback(
            SwerveModel currentPose,
            TimedPose setpoint,
            final double kPCartV,
            final double kPThetaV) {
        final FieldRelativeVelocity velocityError = getFieldRelativeVelocityError(
                currentPose,
                setpoint);
        m_log_velocity_error.log(() -> velocityError);
        final FieldRelativeVelocity u_VFB = new FieldRelativeVelocity(
                kPCartV * velocityError.x(),
                kPCartV * velocityError.y(),
                kPThetaV * velocityError.theta());
        m_log_u_VFB.log(() -> u_VFB);
        return u_VFB;
    }

    public FieldRelativeVelocity fieldRelativeFullFeedback(
            SwerveModel measurement,
            TimedPose setpoint,
            final double kPCart,
            final double kPTheta,
            final double kPCartV,
            final double kPThetaV) {

        // POSITION
        final FieldRelativeVelocity u_XFB = fieldRelativeFeedback(
                measurement,
                setpoint,
                kPCart,
                kPTheta);

        // VELOCITY
        final FieldRelativeVelocity u_VFB = fieldRelativeVelocityFeedback(
                measurement,
                setpoint,
                kPCartV,
                kPThetaV);

        return u_XFB.plus(u_VFB);
    }

    FieldRelativeVelocity getFieldRelativeVelocityError(
            SwerveModel measurement,
            TimedPose setpoint) {
        return SwerveModel.fromTimedPose(setpoint).minus(measurement).velocity();
    }

    public static FieldRelativeDelta fieldRelativeError(Pose2d measurement, TimedPose setpoint) {
        return FieldRelativeDelta.delta(measurement, setpoint.state().getPose());
    }
}
