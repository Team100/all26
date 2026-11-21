package org.team100.lib.logging;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.logging.LoggerFactory.DoubleLogger;

/** Log the sum of all the supply current observations. */
public class TotalCurrentLog {
    public interface Reporter {
        double getSupplyCurrent();
    }

    private final List<Reporter> m_reporters;
    public final DoubleLogger m_logSupplyCurrent;

    public TotalCurrentLog(LoggerFactory parent) {
        m_reporters = new ArrayList<>();
        LoggerFactory log = parent.type(this);
        m_logSupplyCurrent = log.doubleLogger(Level.COMP, "total supply current (A)");
    }

    public void register(Reporter r) {
        m_reporters.add(r);
    }

    public void log() {
        m_logSupplyCurrent
                .log(() -> m_reporters.stream()
                        .map(Reporter::getSupplyCurrent)
                        .reduce(Double::sum)
                        .orElse(0.0));
    }
}
