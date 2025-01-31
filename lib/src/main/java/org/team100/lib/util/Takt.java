package org.team100.lib.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * Takt just caches the FPGA timer, so that all the readers get the same value.
 * 
 * It should be updated in robotPeriodic and nowhere else (except maybe tests).
 */
public class Takt {
    private static double now = 0;

    /**
     * Update the singleton clock.
     * 
     * It's like Timer.getFPGATimestamp() except it only updates once per loop, to
     * avoid injecting clock jitter.
     * 
     * Should be run in Robot.robotPeriodic().
     */
    public static void update() {
        now = actual();
    }

    /** The current Takt time. */
    public static double get() {
        return now;
    }

    /**
     * A few consumers want the actual time; don't use this unless you really do.
     */
    public static double actual() {
        return Timer.getFPGATimestamp();
    }
}
