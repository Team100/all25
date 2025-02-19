package org.team100.lib.logging;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryPoolMXBean;
import java.lang.management.MemoryUsage;
import java.util.HashMap;
import java.util.Map;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.LoggerFactory.LongLogger;

/**
 * Logs stuff about the JVM. Inspired by Advantage Kit's
 * LoggedRobot.GcStatsCollector().
 */
public class JvmLogger implements Glassy {
    private final Map<String, Long> times;
    private final Map<String, Long> counts;
    // LOGGERS
    private final LongLogger m_log_heap;
    private final LongLogger m_log_nonheap;
    private final LongLogger m_log_memory_total;
    private final LongLogger m_log_gc_time;
    private final LongLogger m_log_gc_count;

    public JvmLogger(LoggerFactory parent) {
        LoggerFactory child = parent.child(this);
        times = new HashMap<>();
        counts = new HashMap<>();
        m_log_heap = child.longLogger(Level.DEBUG, "MemoryUsage/heap");
        m_log_nonheap = child.longLogger(Level.TRACE, "MemoryUsage/non-heap");
        m_log_memory_total = child.longLogger(Level.DEBUG, "MemoryPool/total");
        m_log_gc_time = child.longLogger(Level.TRACE, "GCTimeMS/total");
        m_log_gc_count = child.longLogger(Level.TRACE, "GCCounts/total");
    }

    public void logGarbageCollectors() {
        m_log_gc_time.log(() -> garbageCollectorTime());
        m_log_gc_count.log(() -> garbageCollectorCount());
    }

    public void logMemoryPools() {
        m_log_memory_total.log(() -> getPoolUsage());
    }

    public void logMemoryUsage() {
        m_log_heap.log(() -> ManagementFactory.getMemoryMXBean().getHeapMemoryUsage().getUsed());
        m_log_nonheap.log(() -> ManagementFactory.getMemoryMXBean().getNonHeapMemoryUsage().getUsed());
    }

    private long garbageCollectorTime() {
        long accumTime = 0;
        for (GarbageCollectorMXBean bean : ManagementFactory.getGarbageCollectorMXBeans()) {
            String pool = bean.getName();
            times.computeIfAbsent(pool, x -> 0l);
            long collectionTime = bean.getCollectionTime();
            long thisTime = collectionTime - times.get(pool);
            times.put(pool, collectionTime);
            accumTime += thisTime;
        }
        return accumTime;
    }

    private long garbageCollectorCount() {
        long accumCount = 0;
        for (GarbageCollectorMXBean bean : ManagementFactory.getGarbageCollectorMXBeans()) {
            String pool = bean.getName();
            counts.computeIfAbsent(pool, x -> 0l);
            long collectionCount = bean.getCollectionCount();
            long thisCount = collectionCount - counts.get(pool);
            counts.put(pool, collectionCount);
            accumCount += thisCount;
        }
        return accumCount;

    }

    private long getPoolUsage() {
        long accumUsage = 0;
        for (MemoryPoolMXBean bean : ManagementFactory.getMemoryPoolMXBeans()) {
            MemoryUsage usage = bean.getUsage();
            accumUsage += usage.getUsed();
        }
        return accumUsage;
    }

}
