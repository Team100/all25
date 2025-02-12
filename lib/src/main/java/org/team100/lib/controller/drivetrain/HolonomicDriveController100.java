package org.team100.lib.controller.drivetrain;

import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.Model100;

/**
 * PID x, PID y, PID theta
 */
public class HolonomicDriveController100 implements HolonomicFieldRelativeController {
    private final Feedback100 m_xFeedback;
    private final Feedback100 m_yFeedback;
    private final Feedback100 m_thetaFeedback;
    private final Log m_log;

    HolonomicDriveController100(
            Log log,
            Feedback100 xFeedback,
            Feedback100 yFeedback,
            Feedback100 thetaFeedback) {
        m_xFeedback = xFeedback;
        m_yFeedback = yFeedback;
        m_thetaFeedback = thetaFeedback;
        m_log = log;
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
        m_log.measurement.log(() -> measurement);
        m_log.reference.log(() -> currentReference);
        m_log.error.log(() -> currentReference.minus(measurement));

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
        m_log.u_FB.log(() -> u_FB);

        FieldRelativeVelocity u_FF = nextReference.velocity();

        return u_FF.plus(u_FB);
    }

    @Override
    public void reset() {
        m_xFeedback.reset();
        m_yFeedback.reset();
        m_thetaFeedback.reset();
    }
}