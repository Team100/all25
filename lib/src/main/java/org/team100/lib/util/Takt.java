package org.team100.lib.util;

/**
 * Takt just caches the FPGA timer, so that all the readers get the same value.
 * 
 * It should be updated in robotPeriodic and nowhere else (except maybe tests).
 */
public class Takt {
    /** Like the WPI Timer except it uses Takt time, and it is always "running." */
    public static class Timer {
        private double m_startTime;

        public void reset() {
            m_startTime = Takt.get();
        }

        public double get() {
            return Takt.get() - m_startTime;
        }

    }

    /** Current Takt time in seconds. */
    private static double now = actual();

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

    /**
     * The current Takt time in seconds. Even though this is a double, it's ok to
     * test equality, because it is only incremented periodically.
     */
    public static double get() {
        return now;
    }

    /**
     * Curent actual FPGA time in seconds.
     * 
     * A few consumers want the actual time; don't use this unless you really do.
     */
    public static double actual() {
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }
}
