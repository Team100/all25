package org.team100.lib.controller.drivetrain;

import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.logging.LoggerFactory.SwerveModelLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

/**
 * PID x, PID y, PID theta
 */
public class HolonomicDriveController100 implements HolonomicFieldRelativeController {
    /** For testing */
    private static final boolean kPrint = false;
    private final SwerveModelLogger m_log_measurement;
    private final SwerveModelLogger m_log_reference;
    private final SwerveModelLogger m_log_error;
    private final FieldRelativeVelocityLogger m_log_u_FB;

    private final Feedback100 m_xFeedback;
    private final Feedback100 m_yFeedback;
    private final Feedback100 m_thetaFeedback;

    HolonomicDriveController100(
            LoggerFactory parent,
            Feedback100 xFeedback,
            Feedback100 yFeedback,
            Feedback100 thetaFeedback) {
        LoggerFactory child = parent.child("HolonomicFieldRelativeController");
        m_log_reference = child.swerveModelLogger(Level.DEBUG, "reference");
        m_log_measurement = child.swerveModelLogger(Level.DEBUG, "measurement");
        m_log_error = child.swerveModelLogger(Level.DEBUG, "error");
        m_log_u_FB = child.fieldRelativeVelocityLogger(Level.DEBUG, "u_FB");
        m_xFeedback = xFeedback;
        m_yFeedback = yFeedback;
        m_thetaFeedback = thetaFeedback;
    }

    @Override
    public boolean atReference() {
        if (!m_xFeedback.atSetpoint())
            return false;
        if (!m_yFeedback.atSetpoint())
            return false;
        if (!m_thetaFeedback.atSetpoint())
            return false;
        return true;
    }

    /**
     * Makes no attempt to coordinate the axes or provide feasible output.
     */
    @Override
    public FieldRelativeVelocity calculate(
            SwerveModel measurement,
            SwerveModel currentReference,
            SwerveModel nextReference) {
        m_log_measurement.log(() -> measurement);
        m_log_reference.log(() -> currentReference);
        m_log_error.log(() -> currentReference.minus(measurement));

        // feedbacks are velocities
        double xFB = m_xFeedback.calculate(
                Model100.x(measurement.x().x()),
                Model100.x(currentReference.x().x()));
        double yFB = m_yFeedback.calculate(
                Model100.x(measurement.y().x()),
                Model100.x(currentReference.y().x()));
        double thetaFB = m_thetaFeedback.calculate(
                Model100.x(measurement.theta().x()),
                Model100.x(currentReference.theta().x()));

        FieldRelativeVelocity u_FB = new FieldRelativeVelocity(xFB, yFB, thetaFB);
        m_log_u_FB.log(() -> u_FB);

        FieldRelativeVelocity u_FF = nextReference.velocity();

        if (kPrint) {
            Util.printf("measurement %s current %s next %s\n", measurement, currentReference, nextReference);
            Util.printf("ff %s fb %s\n", u_FF, u_FB);
        }

        return u_FF.plus(u_FB);
    }

    @Override
    public void reset() {
        m_xFeedback.reset();
        m_yFeedback.reset();
        m_thetaFeedback.reset();
    }
}