package org.team100.lib.controller.drivetrain;

import org.team100.lib.config.Identity;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.logging.LoggerFactory;

public class HolonomicDriveControllerFactory {

    public static HolonomicFieldRelativeController get(
            LoggerFactory parent,
            HolonomicFieldRelativeController.Log hlog) {
        switch (Identity.instance) {
            case COMP_BOT -> {
                Feedback100 cartesian = new PIDFeedback(parent, 0.5, 0, 0, false, 0.01, 0.1);
                return new HolonomicDriveController100(
                        hlog,
                        cartesian,
                        cartesian,
                        new PIDFeedback(parent, 3.5, 0, 0, true, 0.01, 0.01));
            }
            case SWERVE_ONE -> {
                Feedback100 cartesian = new PIDFeedback(parent, 0.3, 0, 0, false, 0.01, 0.1);
                return new HolonomicDriveController100(
                        hlog,
                        cartesian,
                        cartesian,
                        new PIDFeedback(parent, 3.5, 0, 0, true, 0.01, 0.01));
            }
            case SWERVE_TWO -> {
                return new FullStateDriveController(hlog);
            }
            default -> {
                // these RoboRIO's are have no drivetrains
                Feedback100 cartesian = new PIDFeedback(parent, 3, 1, 0, false, 0.01, 0.1);
                return new HolonomicDriveController100(
                        hlog,
                        cartesian,
                        cartesian,
                        new PIDFeedback(parent, 3.5, 0, 0, true, 0.01, 0.01));
            }
        }
    }

    private HolonomicDriveControllerFactory() {
        //
    }

}
