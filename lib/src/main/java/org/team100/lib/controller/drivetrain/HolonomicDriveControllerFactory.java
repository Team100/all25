package org.team100.lib.controller.drivetrain;

import org.team100.lib.config.Identity;
import org.team100.lib.controller.simple.Controller100;
import org.team100.lib.controller.simple.PIDControllerVeloWPI;
import org.team100.lib.logging.LoggerFactory;

public class HolonomicDriveControllerFactory {

    public static HolonomicFieldRelativeController get(
            LoggerFactory parent,
            HolonomicFieldRelativeController.Log hlog) {
        switch (Identity.instance) {
            case COMP_BOT:
                return new HolonomicDriveController100(parent, hlog, false);
            case SWERVE_ONE:
                return new HolonomicDriveController100(parent, hlog, false);
            case SWERVE_TWO:
                return new FullStateDriveController(hlog);
            case BLANK:
            default:
                return new HolonomicDriveController100(parent, hlog, false);
        }
    }

    public static Controller100 cartesian(LoggerFactory parent) {
        switch (Identity.instance) {
            case COMP_BOT:
                return new PIDControllerVeloWPI(parent, 0.5, 0, 0, false, 0.01, 0.1);
            case SWERVE_ONE:
                return new PIDControllerVeloWPI(parent, 0.3, 0, 0, false, 0.01, 0.1);
            case SWERVE_TWO:
                return new PIDControllerVeloWPI(parent, 2, 0.1, 0.15, false, 0.01, 0.1);
            case BETA_BOT:
                return new PIDControllerVeloWPI(parent, 3, 2, 0, false, 0.01, 0.1);
            case BLANK:
                // for testing
                return new PIDControllerVeloWPI(parent, 3, 1, 0, false, 0.01, 0.1);
            default:
                // these RoboRIO's are have no drivetrains
                return new PIDControllerVeloWPI(parent, 1, 0.0, 0.0, false, 0.01, 0.1);
        }
    }

    public static Controller100 theta(LoggerFactory parent) {
        // 0.01 rad = 0.5 degrees
        return new PIDControllerVeloWPI(parent, 3.5, 0, 0, true, 0.01, 0.01);
    }

    public static Controller100 omega(LoggerFactory parent) {
        // 0.01 rad/s = 0.5 degrees/s tolerance
        return new PIDControllerVeloWPI(parent, 1.5, 0, 0, false, 0.01, 0.01);
    }

    private HolonomicDriveControllerFactory() {
        //
    }

}
