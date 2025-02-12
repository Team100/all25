package org.team100.lib.controller.drivetrain;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.follower.TrajectoryFollower;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.MathUtil;

/**
 * Three independent axes of proportional position and velocity control, with
 * setpoint velocity feedforward.
 */
public class FullStateDriveController implements HolonomicFieldRelativeController {
    private final double m_xK1; // position
    private final double m_xK2; // velocity
    private final double m_thetaK1; // position
    private final double m_thetaK2; // velocity
    private final double m_xTolerance; // 1 cm
    private final double m_thetaTolerance; // 1 degree
    private final double m_xDotTolerance; // 1 cm/s
    private final double m_omegaTolerance; // 1 degree/s

    private final Log m_log;

    private boolean m_atSetpoint = false;

    TrajectoryFollower follower;

    public FullStateDriveController(
            LoggerFactory parent,
            Log log,
            double xK1,
            double xK2,
            double thetaK1,
            double thetaK2,
            double xTolerance,
            double thetaTolerance,
            double xDotTolerance,
            double omegaTolerance) {
        m_log = log;
        m_xK1 = xK1; // position
        m_xK2 = xK2; // velocity
        m_thetaK1 = thetaK1; // position
        m_thetaK2 = thetaK2; // velocity
        m_xTolerance = xTolerance; // 1 cm
        m_thetaTolerance = thetaTolerance; // 1 degree
        m_xDotTolerance = xDotTolerance; // 1 cm/s
        m_omegaTolerance = omegaTolerance;

        follower = new TrajectoryFollower(parent, xK1, thetaK1, xK2, thetaK2);
    }

    public static FullStateDriveController getDefault(LoggerFactory parent, Log log) {
        return new FullStateDriveController(
                parent,
                log,
                4, // position
                0.25, // velocity
                4, // theta
                0.25, // omega
                0.01, // position tolerance
                0.02, // theta tolerance
                0.01, // velocity tolerance
                0.02); // omega tolerance
    }

    @Override
    public FieldRelativeVelocity calculate(
            SwerveModel measurement,
            SwerveModel currentReference,
            SwerveModel nextReference) {
        m_log.measurement.log(() -> measurement);
        m_log.reference.log(() -> currentReference);
        m_log.error.log(() -> currentReference.minus(measurement));

        m_atSetpoint = true;

        double xFB = calculateFB(m_xK1, m_xK2, m_xTolerance, m_xDotTolerance,
                measurement.x(), currentReference.x(), x -> x);
        double yFB = calculateFB(m_xK1, m_xK2, m_xTolerance, m_xDotTolerance,
                measurement.y(), currentReference.y(), x -> x);
        double thetaFB = calculateFB(m_thetaK1, m_thetaK2, m_thetaTolerance, m_omegaTolerance,
                measurement.theta(), currentReference.theta(), MathUtil::angleModulus);

        // FieldRelativeVelocity u_FB = new FieldRelativeVelocity(xFB, yFB, thetaFB);

        FieldRelativeVelocity u_FB = follower.fullFeedback(measurement, currentReference);

        m_log.u_FB.log(() -> u_FB);

        // FieldRelativeVelocity u_FF = nextReference.velocity();
        FieldRelativeVelocity u_FF = follower.feedforward(nextReference);

        return u_FF.plus(u_FB);
    }

    private double calculateFB(
            double k1,
            double k2,
            double xTolerance,
            double xDotTolerance,
            Model100 measurement,
            Model100 setpoint,
            DoubleUnaryOperator modulus) {
        double xError = modulus.applyAsDouble(setpoint.x() - measurement.x());
        double xDotError = setpoint.v() - measurement.v();
        m_atSetpoint &= Math.abs(xError) < xTolerance
                && Math.abs(xDotError) < xDotTolerance;
        return k1 * xError + k2 * xDotError;
    }

    /** True if the most recent call to calculate() was at the setpoint. */
    @Override
    public boolean atReference() {
        return m_atSetpoint;
    }

    @Override
    public void reset() {
        m_atSetpoint = false;
    }
}
