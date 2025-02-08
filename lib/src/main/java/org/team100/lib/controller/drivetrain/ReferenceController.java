package org.team100.lib.controller.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

/**
 * Drives based on a reference time series.
 * 
 * If the current reference velocity is zero, this waits a bit for the wheels to
 * align to the next reference direction, eliminating the little wiggle that
 * happens with uncoordinated steer/drive commands.
 */
public class ReferenceController {
    /**
     * @param v      desired velocity
     * @param atRest if true, actuate steering only
     */
    public record Output(FieldRelativeVelocity v, boolean atRest) {
    }

    public Output calculate(SwerveModel measurement) {
        return null;

    }

}
