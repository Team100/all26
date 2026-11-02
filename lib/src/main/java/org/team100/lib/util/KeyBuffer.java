package org.team100.lib.util;

import java.util.NavigableSet;
import java.util.TreeSet;

/**
 * Like TimeInterpolatableBuffer100 but with no value.
 * 
 * Used to keep track of keys whose value is stored elsewhere.
 * 
 * NOTE: the actual keys use microseconds, which are not correctly
 * represented as doubles.
 * TODO: instead use the actual microseconds here.
 */
public class KeyBuffer {
    /** Length of the buffer in seconds */
    private final double m_historyS;
    private final NavigableSet<Double> m_timestamps;

    public KeyBuffer(double historyS) {
        m_historyS = historyS;
        m_timestamps = new TreeSet<>();
    }

    public boolean contains(double x) {
        return m_timestamps.contains(x);
    }

    public void add(double x) {
        while (!m_timestamps.isEmpty()) {
            double oldestTimeS = m_timestamps.first();
            double oldestAgeS = x - oldestTimeS;
            // if oldest is younger than the history limit, we're done
            if (oldestAgeS < m_historyS)
                break;
            m_timestamps.remove(oldestTimeS);
        }
        m_timestamps.add(x);
    }

    public void clear() {
        m_timestamps.clear();
    }

    public double last() {
        return m_timestamps.last();
    }
}
