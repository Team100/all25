package org.team100.lib.controller.drivetrain;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

public class FullStateDriveController implements HolonomicFieldRelativeController {

    private final SwerveController follower;

    public FullStateDriveController(
            LoggerFactory parent,
            double xK1,
            double xK2,
            double thetaK1,
            double thetaK2,
            double xTolerance,
            double thetaTolerance,
            double xDotTolerance,
            double omegaTolerance) {
        follower = new SwerveController(parent,
                xK1, thetaK1, xK2, thetaK2,
                xTolerance, thetaTolerance, xDotTolerance, omegaTolerance);
    }

    public static FullStateDriveController getDefault(LoggerFactory parent) {
        return new FullStateDriveController(
                parent,
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
        return follower.calculate(measurement, currentReference, nextReference);
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
