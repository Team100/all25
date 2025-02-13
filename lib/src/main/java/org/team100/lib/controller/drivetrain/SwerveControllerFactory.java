package org.team100.lib.controller.drivetrain;

import org.team100.lib.logging.LoggerFactory;

/**
 * Known-good controller settings.
 * 
 * TODO: combine with holonomic drive controller factory.
 */
public class SwerveControllerFactory {

    public static SwerveController fieldRelativeFancyPIDF(LoggerFactory log) {
        return new SwerveController(log, 2.4, 1.3, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02);
    }

    public static SwerveController fieldRelativeGoodPIDF(LoggerFactory log) {
        return new SwerveController(log, 1, 1.3, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02);
    }

    public static SwerveController autoFieldRelativePIDF(LoggerFactory log) {
        return new SwerveController(log, 1, 1.3, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02);
    }

    public static SwerveController fieldRelativeFfOnly(LoggerFactory log) {
        return new SwerveController(log, 0, 0, 0, 0, 0.01, 0.02, 0.01, 0.02);
    }

    public static SwerveController testFieldRelativePIDF(LoggerFactory log) {
        return new SwerveController(log, 2.4, 2.4, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02);
    }

    public static SwerveController testFieldRelativeFFOnly(LoggerFactory log) {
        return new SwerveController(log, 0, 0, 0, 0, 0.01, 0.02, 0.01, 0.02);
    }

    private SwerveControllerFactory() {
        // don't call this
    }
}
