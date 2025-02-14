package org.team100.lib.controller.drivetrain;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.FieldRelativeDeltaLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.logging.LoggerFactory.SwerveModelLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

/**
 * Velocity feedforward, proportional feedback on position and velocity.
 */
public class FullStateSwerveController implements SwerveController {
    private final SwerveModelLogger m_log_measurement;
    private final SwerveModelLogger m_log_currentReference;
    private final SwerveModelLogger m_log_nextReference;
    private final FieldRelativeVelocityLogger m_log_u_FF;
    private final FieldRelativeDeltaLogger m_log_position_error;
    private final FieldRelativeVelocityLogger m_log_u_FB;
    private final FieldRelativeVelocityLogger m_log_velocity_error;
    private final FieldRelativeVelocityLogger m_log_u_VFB;

    private final double m_kPCart;
    private final double m_kPTheta;
    private final double m_kPCartV;
    private final double m_kPThetaV;
    private final double m_xTolerance;
    private final double m_thetaTolerance;
    private final double m_xDotTolerance;
    private final double m_omegaTolerance;

    private boolean m_atReference;

    public FullStateSwerveController(
            LoggerFactory parent,
            double kPCart,
            double kPTheta,
            double kPCartV,
            double kPThetaV,
            double xTolerance,
            double thetaTolerance,
            double xDotTolerance,
            double omegaTolerance) {
        LoggerFactory log = parent.child(this);
        m_log_measurement = log.swerveModelLogger(Level.DEBUG, "measurement");
        m_log_currentReference = log.swerveModelLogger(Level.DEBUG, "currentReference");
        m_log_nextReference = log.swerveModelLogger(Level.DEBUG, "nextReference");
        m_log_u_FF = log.fieldRelativeVelocityLogger(Level.TRACE, "u_FF");
        m_log_position_error = log.fieldRelativeDeltaLogger(Level.TRACE, "positionError");
        m_log_u_FB = log.fieldRelativeVelocityLogger(Level.TRACE, "u_FB");
        m_log_velocity_error = log.fieldRelativeVelocityLogger(Level.TRACE, "velocityError");
        m_log_u_VFB = log.fieldRelativeVelocityLogger(Level.TRACE, "u_VFB");

        m_kPCart = kPCart;
        m_kPTheta = kPTheta;
        m_kPCartV = kPCartV;
        m_kPThetaV = kPThetaV;

        m_xTolerance = xTolerance;
        m_thetaTolerance = thetaTolerance;
        m_xDotTolerance = xDotTolerance;
        m_omegaTolerance = omegaTolerance;
    }

    @Override
    public FieldRelativeVelocity calculate(
            SwerveModel measurement,
            SwerveModel currentReference,
            SwerveModel nextReference) {
        m_log_measurement.log(() -> measurement);
        m_log_currentReference.log(() -> currentReference);
        m_log_nextReference.log(() -> nextReference);
        FieldRelativeVelocity u_FF = feedforward(nextReference);
        FieldRelativeVelocity u_FB = fullFeedback(measurement, currentReference);
        return u_FF.plus(u_FB);
    }

    @Override
    public boolean atReference() {
        return m_atReference;
    }

    @Override
    public void reset() {
        //
    }

    ///////////////////////////////////////////////
    //
    // package-private for testing

    FieldRelativeVelocity feedforward(SwerveModel nextReference) {
        m_log_u_FF.log(() -> nextReference.velocity());
        return nextReference.velocity();
    }

    FieldRelativeVelocity fullFeedback(SwerveModel measurement, SwerveModel currentReference) {
        // atReference is updated below
        m_atReference = true;
        FieldRelativeVelocity u_XFB = positionFeedback(measurement, currentReference);
        FieldRelativeVelocity u_VFB = velocityFeedback(measurement, currentReference);
        return u_XFB.plus(u_VFB);
    }

    FieldRelativeVelocity positionFeedback(SwerveModel measurement, SwerveModel currentReference) {
        // wraps heading
        FieldRelativeDelta positionError = positionError(measurement, currentReference);
        m_atReference &= positionError.getTranslation().getNorm() < m_xTolerance
                && Math.abs(positionError.getRotation().getRadians()) < m_thetaTolerance;
        FieldRelativeVelocity u_FB = new FieldRelativeVelocity(
                m_kPCart * positionError.getX(),
                m_kPCart * positionError.getY(),
                m_kPTheta * positionError.getRotation().getRadians());
        m_log_u_FB.log(() -> u_FB);
        return u_FB;
    }

    FieldRelativeVelocity velocityFeedback(SwerveModel currentPose, SwerveModel currentReference) {
        final FieldRelativeVelocity velocityError = velocityError(currentPose, currentReference);
        m_atReference &= velocityError.norm() < m_xDotTolerance
                && Math.abs(velocityError.angle().orElse(GeometryUtil.kRotationZero).getRadians()) < m_omegaTolerance;

        final FieldRelativeVelocity u_VFB = new FieldRelativeVelocity(
                m_kPCartV * velocityError.x(),
                m_kPCartV * velocityError.y(),
                m_kPThetaV * velocityError.theta());
        m_log_u_VFB.log(() -> u_VFB);
        return u_VFB;
    }

    FieldRelativeDelta positionError(SwerveModel measurement, SwerveModel currentReference) {
        FieldRelativeDelta positionError = FieldRelativeDelta.delta(measurement.pose(), currentReference.pose());
        m_log_position_error.log(() -> positionError);
        return positionError;
    }

    FieldRelativeVelocity velocityError(SwerveModel measurement, SwerveModel currentReference) {
        FieldRelativeVelocity velocityError = currentReference.minus(measurement).velocity();
        m_log_velocity_error.log(() -> velocityError);
        return velocityError;
    }

}
