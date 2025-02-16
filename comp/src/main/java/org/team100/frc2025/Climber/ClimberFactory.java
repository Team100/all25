package org.team100.frc2025.Climber;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;

public class ClimberFactory {
    
    public static Climber get(LoggerFactory logger) {
        switch (Identity.instance) {
            case TEST_BOARD_B0:
                return new Climber(logger,18);
            default:
                return null;
        }
    }
}
