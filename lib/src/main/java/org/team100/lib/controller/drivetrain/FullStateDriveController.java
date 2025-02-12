package org.team100.lib.controller.drivetrain;

import org.team100.lib.follower.FollowerController;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

public class FullStateDriveController implements HolonomicFieldRelativeController {
    private final Log m_log;
    FollowerController follower;

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
        follower = new FollowerController(parent,
                xK1, thetaK1, xK2, thetaK2,
                xTolerance, thetaTolerance, xDotTolerance, omegaTolerance);
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
        FieldRelativeVelocity u_FB = follower.fullFeedback(measurement, currentReference);
        m_log.u_FB.log(() -> u_FB);
        FieldRelativeVelocity u_FF = follower.feedforward(nextReference);
        return u_FF.plus(u_FB);
    }

    /** True if the most recent call to calculate() was at the setpoint. */
    @Override
    public boolean atReference() {
        return follower.atReference();
    }

    @Override
    public void reset() {
    }
}
