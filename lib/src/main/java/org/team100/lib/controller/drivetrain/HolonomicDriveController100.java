package org.team100.lib.controller.drivetrain;

import org.team100.lib.controller.simple.Controller100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.Model100;

/**
 * PID x, PID y, PID theta, and (optionally) PID omega.
 */
public class HolonomicDriveController100 implements HolonomicFieldRelativeController {
    private final Controller100 m_xController;
    private final Controller100 m_yController;
    private final Controller100 m_thetaController;
    private final Controller100 m_omegaController;
    private final boolean m_useOmega;
    private final Log m_log;

    /**
     * Use the factory.
     * 
     * @param useOmega include omega feedback
     */
    HolonomicDriveController100(LoggerFactory parent, Log log, boolean useOmega) {
        m_xController = HolonomicDriveControllerFactory.cartesian(parent);
        m_yController = HolonomicDriveControllerFactory.cartesian(parent);
        m_thetaController = HolonomicDriveControllerFactory.theta(parent);
        m_omegaController = HolonomicDriveControllerFactory.omega(parent);
        m_useOmega = useOmega;
        m_log = log;
    }

    @Override
    public boolean atReference() {
        if (!m_xController.atSetpoint())
            return false;
        if (!m_yController.atSetpoint())
            return false;
        if (!m_thetaController.atSetpoint())
            return false;
        if (!m_useOmega)
            return true;
        return m_omegaController.atSetpoint();
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
        double xFB = m_xController.calculate(
                Model100.x(measurement.x().x()),
                Model100.x(reference.x().x())).v();
        double yFB = m_yController.calculate(
                Model100.x(measurement.y().x()),
                Model100.x(reference.y().x())).v();
        double thetaFB = m_thetaController.calculate(
                Model100.x(measurement.theta().x()),
                Model100.x(reference.theta().x())).v();
        double omegaFB = 0.0;
        if (m_useOmega) {
            omegaFB = m_omegaController.calculate(Model100.x(measurement.theta().v()),
                    Model100.x(reference.theta().v())).v();
        }
        FieldRelativeVelocity u_FB = new FieldRelativeVelocity(xFB, yFB, thetaFB + omegaFB);
        m_log.u_FB.log(() -> u_FB);
        return u_FF.plus(u_FB);
    }

    @Override
    public void reset() {
        m_xController.reset();
        m_yController.reset();
        m_thetaController.reset();
        if (m_useOmega)
            m_omegaController.reset();
    }
}