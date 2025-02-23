package org.team100.lib.logging;

import org.team100.lib.logging.primitive.NTPrimitiveLogger;
import org.team100.lib.logging.primitive.PrimitiveLogger;

import com.ctre.phoenix6.SignalLogger;

/** Logging singleton */
public class Logging {
    private static final Logging instance = new Logging();

    private PrimitiveLogger ntLogger;
    private Level m_level;

    /**
     * root is "field", with a ".type"->"Field2d" entry as required by glass.
     */
    public final LoggerFactory fieldLogger;
    /** root is "log". */
    public final LoggerFactory rootLogger;

    /**
     * Clients should use the static instance, not the constructor.
     */
    private Logging() {
        // this will be overridden by {@link LogLevelPoller}
        m_level = Level.COMP;

        ntLogger = new NTPrimitiveLogger();
        fieldLogger = new LoggerFactory(() -> m_level, "field", ntLogger);
        rootLogger = new LoggerFactory(() -> m_level, "log", ntLogger);

        fieldLogger.stringLogger(Level.COMP, ".type").log(() -> "Field2d");

        // turn off the CTRE log we never use
        SignalLogger.enableAutoLogging(false);
    }

    public int keyCount() {
        if (ntLogger != null)
            return ntLogger.keyCount();
        return 0;
    }

    public void periodic() {
    }

    public void setLevel(Level level) {
        m_level = level;
    }

    public Level getLevel() {
        return m_level;
    }

    public static Logging instance() {
        return instance;
    }
}