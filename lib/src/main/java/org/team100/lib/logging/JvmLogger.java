package org.team100.lib.logging;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryPoolMXBean;
import java.util.HashMap;
import java.util.Map;

import org.team100.lib.logging.LoggerFactory.LongLogger;

/**
 * Logs stuff about the JVM. Inspired by Advantage Kit's
 * LoggedRobot.GcStatsCollector().
 */
public class JvmLogger  {
    private final LoggerFactory m_log;
    private final LongLogger m_log_heap;
    private final LongLogger m_log_nonheap;
    private final Map<String, LongLogger> m_log_gc_times = new HashMap<>();
    private final Map<String, LongLogger> m_log_gc_counts = new HashMap<>();
    private final Map<String, LongLogger> m_log_memory = new HashMap<>();

    public JvmLogger(LoggerFactory parent) {
        m_log = parent.type(this);
        m_log_heap = m_log.longLogger(Level.DEBUG, "MemoryUsage/heap");
        m_log_nonheap = m_log.longLogger(Level.TRACE, "MemoryUsage/non-heap");
    }

    public void logGarbageCollectors() {
        if (!Logging.instance().isAllowed(Level.TRACE)) {
            // don't do any work if we're not going to log it.
            return;
        }
        for (GarbageCollectorMXBean bean : ManagementFactory.getGarbageCollectorMXBeans()) {
            m_log_gc_counts.computeIfAbsent(
                    bean.getName(),
                    (x) -> m_log.longLogger(Level.TRACE, "GCCount/" + x))
                    .log(() -> bean.getCollectionCount());
            m_log_gc_times.computeIfAbsent(
                    bean.getName(),
                    (x) -> m_log.longLogger(Level.TRACE, "GCTime_ms/" + x))
                    .log(() -> bean.getCollectionTime());
        }
    }

    public void logMemoryPools() {
        if (!Logging.instance().isAllowed(Level.TRACE)) {
            // don't do any work if we're not going to log it.
            return;
        }
        for (MemoryPoolMXBean bean : ManagementFactory.getMemoryPoolMXBeans()) {
            m_log_memory.computeIfAbsent(
                    bean.getName(),
                    (x) -> m_log.longLogger(Level.TRACE, "Memory/" + x))
                    .log(() -> bean.getUsage().getUsed());
        }
    }

    public void logMemoryUsage() {
        m_log_heap.log(() -> ManagementFactory.getMemoryMXBean().getHeapMemoryUsage().getUsed());
        m_log_nonheap.log(() -> ManagementFactory.getMemoryMXBean().getNonHeapMemoryUsage().getUsed());
    }

}
