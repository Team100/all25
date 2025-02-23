package org.team100.lib.logging;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryPoolMXBean;
import java.util.HashMap;
import java.util.Map;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.LoggerFactory.LongLogger;

/**
 * Logs stuff about the JVM. Inspired by Advantage Kit's
 * LoggedRobot.GcStatsCollector().
 */
public class JvmLogger implements Glassy {
    private final LoggerFactory child;
    private final LongLogger m_log_heap;
    private final LongLogger m_log_nonheap;
    private final Map<String, LongLogger> m_log_gc_times = new HashMap<>();
    private final Map<String, LongLogger> m_log_gc_counts = new HashMap<>();
    private final Map<String, LongLogger> m_log_memory = new HashMap<>();

    public JvmLogger(LoggerFactory parent) {
        child = parent.child(this);
        m_log_heap = child.longLogger(Level.DEBUG, "MemoryUsage/heap");
        m_log_nonheap = child.longLogger(Level.TRACE, "MemoryUsage/non-heap");
    }

    public void logGarbageCollectors() {
        if (!Logging.instance().getLevel().admit(Level.TRACE)) {
            // don't do any work if we're not going to log it.
            return;
        }
        for (GarbageCollectorMXBean bean : ManagementFactory.getGarbageCollectorMXBeans()) {
            m_log_gc_counts.computeIfAbsent(
                    bean.getName(),
                    (x) -> child.longLogger(Level.TRACE, "GCCount/" + x))
                    .log(() -> bean.getCollectionCount());
            m_log_gc_times.computeIfAbsent(
                    bean.getName(),
                    (x) -> child.longLogger(Level.TRACE, "GCTime_ms/" + x))
                    .log(() -> bean.getCollectionTime());
        }
    }

    public void logMemoryPools() {
        if (!Logging.instance().getLevel().admit(Level.TRACE)) {
            // don't do any work if we're not going to log it.
            return;
        }
        for (MemoryPoolMXBean bean : ManagementFactory.getMemoryPoolMXBeans()) {
            m_log_memory.computeIfAbsent(
                    bean.getName(),
                    (x) -> child.longLogger(Level.TRACE, "Memory/" + x))
                    .log(() -> bean.getUsage().getUsed());
        }
    }

    public void logMemoryUsage() {
        m_log_heap.log(() -> ManagementFactory.getMemoryMXBean().getHeapMemoryUsage().getUsed());
        m_log_nonheap.log(() -> ManagementFactory.getMemoryMXBean().getNonHeapMemoryUsage().getUsed());
    }

}
