package org.team100.lib.util;

import java.util.Collection;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;

import org.wpilib.math.interpolation.Interpolatable;

/**
 * like TimeInterpolatableBuffer but for discrete time measured in microseconds.
 * 
 * TODO: someday combine these two classes
 */
public class DiscreteTimeInterpolatableBuffer100<T extends Interpolatable<T>> {
    private static final boolean DEBUG = false;

    /** Length of the buffer in seconds */
    private final long m_historyS;
    /** Key is timestamp in seconds */
    private final NavigableMap<Long, T> m_pastSnapshots;

    public DiscreteTimeInterpolatableBuffer100(
            long historyS,
            long timeS,
            T initialValue) {
        m_pastSnapshots = new TreeMap<>();
        m_historyS = historyS;
        m_pastSnapshots.put(timeS, initialValue);
    }

    /** For the read-only copy. */
    public DiscreteTimeInterpolatableBuffer100(
            Collection<Map.Entry<Long, T>> entries,
            long historyS) {
        m_pastSnapshots = new TreeMap<>();
        for (Map.Entry<Long, T> e : entries) {
            m_pastSnapshots.put(e.getKey(), e.getValue());
        }
        m_historyS = historyS;
    }

    /**
     * Remove stale entries and add the new one.
     */
    public void put(long timeS, T value) {
        if (DEBUG) {
            System.out.printf("put %f %s\n", timeS, value);
        }
        while (!m_pastSnapshots.isEmpty()) {
            Entry<Long, T> oldest = m_pastSnapshots.firstEntry();
            Long oldestTimeS = oldest.getKey();
            long oldestAgeS = timeS - oldestTimeS;
            // if oldest is younger than the history limit, we're done
            if (oldestAgeS < m_historyS)
                break;
            m_pastSnapshots.remove(oldestTimeS);
            if (DEBUG)
                System.out.printf("removed %f\n", oldestTimeS);
        }
        m_pastSnapshots.put(timeS, value);
    }

    /**
     * Remove all entries and add the new one.
     */
    public void reset(long timeS, T value) {
        if (DEBUG) {
            System.out.printf("reset %f %s\n", timeS, value);
        }
        m_pastSnapshots.clear();
        m_pastSnapshots.put(timeS, value);
    }

    /**
     * Sample the buffer at the given time. May return an entry exactly, if the
     * timestamp matches a known timestamp exactly. Otherwise returns an
     * interpolated value.
     */
    public T get(long timeSeconds) {
        // Special case for when the requested time is the same as a sample
        T nowEntry = m_pastSnapshots.get(timeSeconds);
        if (nowEntry != null) {
            if (DEBUG) {
                System.out.printf("record for now %.2f\n", timeSeconds);
            }
            return nowEntry;
        }
        Entry<Long, T> topBound = m_pastSnapshots.ceilingEntry(timeSeconds);
        Entry<Long, T> bottomBound = m_pastSnapshots.floorEntry(timeSeconds);

        if (topBound == null && bottomBound == null) {
            throw new IllegalStateException();
        }
        // Return the opposite bound if the other is null
        if (topBound == null) {
            return bottomBound.getValue();
        }
        if (bottomBound == null) {
            return topBound.getValue();
        }

        // If both bounds exist, interpolate between them.
        // Because T is between [0, 1], we want the ratio of
        // (the difference between the current time and bottom bound) and (the
        // difference between top and bottom bounds).

        long timeSinceBottom = timeSeconds - bottomBound.getKey();
        long timeSpan = topBound.getKey() - bottomBound.getKey();
        double timeFraction = (double)(timeSinceBottom) / (double)(timeSpan);
        if (DEBUG) {
            System.out.printf("timeSeconds %f bottom %f top %f\n",
                    timeSeconds, bottomBound.getKey(), topBound.getKey());

            System.out.printf("interpolate %f\n", timeFraction);
        }
        return bottomBound.getValue().interpolate(
                topBound.getValue(), timeFraction);
    }

    /** Items after the given timestamp, optionally inclusive */
    public SortedMap<Long, T> tailMap(long t, boolean inclusive) {
        return m_pastSnapshots.tailMap(t, inclusive);
    }

    /** True if the timestamp is older than the history window. */
    public boolean tooOld(long timestampS) {
        Long newestSeenS = m_pastSnapshots.lastKey();
        long oldestAcceptableS = newestSeenS - m_historyS;
        return timestampS < oldestAcceptableS;
    }

    /** Return entry for key strictly smaller than t. */
    public Entry<Long, T> lowerEntry(long t) {
        return m_pastSnapshots.lowerEntry(t);
    }

    /** Return entry for key strictly greater than t */
    public Entry<Long, T> higherEntry(long t) {
        return m_pastSnapshots.higherEntry(t);
    }

    public int size() {
        return m_pastSnapshots.size();
    }

    /** Key of the most-recent snapshot. */
    public long lastKey() {
        return m_pastSnapshots.lastKey();
    }

    public Set<Map.Entry<Long, T>> entrySet() {
        return m_pastSnapshots.entrySet();
    }

    public Map<Long, T> all() {
        return m_pastSnapshots;
    }

    /** Print the entire buffer */
    public void dump() {
        for (var x : entrySet()) {
            System.out.printf("%s: %s\n", x.getKey(), x.getValue());
        }
    }
}