package org.team100.lib.controller.drivetrain;

import org.team100.lib.controller.simple.Controller100;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.Model100;

/**
 * PID x, PID y, PID theta, and (optionally) PID omega.
 */
public class HolonomicDriveController100 implements HolonomicFieldRelativeController {
    private final Feedback100 m_xFeedback;
    private final Feedback100 m_yFeedback;
    private final Feedback100 m_thetaFeedback;
    private final Feedback100 m_omegaFeedback;
    private final boolean m_useOmega;
    private final Log m_log;

    /**
     * Use the factory.
     * 
     * @param useOmega include omega feedback
     */
    HolonomicDriveController100(LoggerFactory parent, Log log, boolean useOmega) {
        m_xFeedback = HolonomicDriveControllerFactory.cartesian(parent);
        m_yFeedback = HolonomicDriveControllerFactory.cartesian(parent);
        m_thetaFeedback = HolonomicDriveControllerFactory.theta(parent);
        m_omegaFeedback = HolonomicDriveControllerFactory.omega(parent);
        m_useOmega = useOmega;
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
        if (!m_useOmega)
            return true;
        return m_omegaFeedback.atSetpoint();
    }

    /**
     * Makes no attempt to coordinate the axes or provide feasible output.
     */
    @Override
    public FieldRelativeVelocity calculate(SwerveModel measurement, SwerveModel reference) {
        m_log.measurement.log(() -> measurement);
        m_log.reference.log(() -> reference);
        m_log.error.log(() -> reference.minus(measurement));

        FieldRelativeVelocity u_FF = reference.velocity();

        // feedbacks are velocities
        double xFB = m_xFeedback.calculate(
                Model100.x(measurement.x().x()),
                Model100.x(reference.x().x()));
        double yFB = m_yFeedback.calculate(
                Model100.x(measurement.y().x()),
                Model100.x(reference.y().x()));
        double thetaFB = m_thetaFeedback.calculate(
                Model100.x(measurement.theta().x()),
                Model100.x(reference.theta().x()));
        double omegaFB = 0.0;
        if (m_useOmega) {
            omegaFB = m_omegaFeedback.calculate(Model100.x(measurement.theta().v()),
                    Model100.x(reference.theta().v()));
        }
        FieldRelativeVelocity u_FB = new FieldRelativeVelocity(xFB, yFB, thetaFB + omegaFB);
        m_log.u_FB.log(() -> u_FB);
        return u_FF.plus(u_FB);
    }

    @Override
    public void reset() {
        m_xFeedback.reset();
        m_yFeedback.reset();
        m_thetaFeedback.reset();
        if (m_useOmega)
            m_omegaFeedback.reset();
    }
}