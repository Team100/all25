package org.team100.lib.localization;

import org.team100.lib.util.Util;

/**
 * Publishes AprilTag Blip24 sightings on Network Tables, just like real
 * cameras would.
 */
public class SimulatedTagDetector {
    private static final boolean DEBUG = false;

    public void periodic() {
        if (DEBUG)
            Util.println("simulated tag detector");
    }

}
