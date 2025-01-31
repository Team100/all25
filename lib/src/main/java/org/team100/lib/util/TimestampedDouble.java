package org.team100.lib.util;

/**
 * Use this for measurements that include timestamps.
 * 
 * Network Tables uses a time basis of FPGA microseconds, which can be found at
 * Takt.get().
 * 
 * We standardize on seconds, which can be found at Takt.get().
 * 
 * @see https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html#timestamps
 */
public class TimestampedDouble {
    private final double m_value;
    private final double m_timeS;

    /** Supply time in seconds */
    public TimestampedDouble(double value, double timeS) {
        m_value = value;
        m_timeS = timeS;
    }

    /** use the current time */
    public TimestampedDouble(double value) {
        m_value = value;
        m_timeS = Takt.get();
    }

    public double getValue() {
        return m_value;
    }

    public double getTimeS() {
        return m_timeS;
    }
}
