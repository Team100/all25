package org.team100.lib.logging;

import org.team100.lib.logging.primitive.DummySender;
import org.team100.lib.logging.primitive.NTPrimitiveLogger;
import org.team100.lib.logging.primitive.PrimitiveLogger;
import org.team100.lib.logging.primitive.UdpPrimitiveLogger;
import org.team100.lib.logging.primitive.UdpSender;
import org.team100.lib.util.Util;

import com.ctre.phoenix6.SignalLogger;

/** Logging singleton */
public class LoggingWithUDPOption {
    private static final boolean USE_UDP_LOGGING = false;
    private static final boolean USE_REAL_UDP = false;

    private static final LoggingWithUDPOption instance = new LoggingWithUDPOption();

    private UdpPrimitiveLogger udpLogger;
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
    private LoggingWithUDPOption() {
        // this will be overridden by {@link LogLevelPoller}
        m_level = Level.COMP;
        if (USE_UDP_LOGGING) {
            Util.warn("=======================================");
            Util.warn("Using UDP network logging!");
            Util.warn("You must have a log listener connected!");
            Util.warn("=======================================");
            if (USE_REAL_UDP) {
                udpLogger = new UdpPrimitiveLogger(
                        UdpSender.data(),
                        UdpSender.meta());
            } else {
                udpLogger = new UdpPrimitiveLogger(
                        new DummySender(),
                        new DummySender());
            }
            fieldLogger = new LoggerFactory(() -> m_level, "field", udpLogger);
            rootLogger = new LoggerFactory(() -> m_level, "log", udpLogger);
        } else {
            ntLogger = new NTPrimitiveLogger();
            fieldLogger = new LoggerFactory(() -> m_level, "field", ntLogger);
            rootLogger = new LoggerFactory(() -> m_level, "log", ntLogger);
        }

        fieldLogger.stringLogger(Level.COMP, ".type").log(() -> "Field2d");

        // turn off the CTRE log we never use
        SignalLogger.enableAutoLogging(false);
    }

    public int keyCount() {
        if (udpLogger != null)
            return udpLogger.keyCount();
        if (ntLogger != null)
            return ntLogger.keyCount();
        return 0;
    }

    public void periodic() {
        if (udpLogger != null)
            udpLogger.periodic();
    }

    public void setLevel(Level level) {
        m_level = level;
    }

    public Level getLevel() {
        return m_level;
    }

    public static LoggingWithUDPOption instance() {
        return instance;
    }
}