package org.team100.lib.util;

import java.util.Map;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;

import org.wpilib.math.interpolation.Interpolatable;

/**
 * Uses an Interpolator to provide interpolated sampling with a history limit.
 * 
 * The buffer is never empty, so get() always returns *something*.
 * 
 * I tried making this "maybe empty" but it makes the client handle empty cases
 * that never actually occur (at startup, the pose really is unknown (so we
 * use zero) and the module positions really are zero). It's simpler not to do
 * that.
 */
public class TimeInterpolatableBuffer100<T extends Interpolatable<T>> {
    private static final boolean DEBUG = false;

    /** Length of the buffer in seconds */
    private final double m_historyS;
    /** Key is timestamp in seconds */
    private final NavigableMap<Double, T> m_pastSnapshots;

    public TimeInterpolatableBuffer100(
            double historyS,
            double timeS,
            T initialValue) {
        m_pastSnapshots = new TreeMap<>();
        m_historyS = historyS;
        m_pastSnapshots.put(timeS, initialValue);
    }

    /** For the read-only copy. */
    public TimeInterpolatableBuffer100(
            Set<Map.Entry<Double, T>> entries,
            double historyS) {
        m_pastSnapshots = new TreeMap<>();
        for (Map.Entry<Double, T> e : entries) {
            m_pastSnapshots.put(e.getKey(), e.getValue());
        }
        m_historyS = historyS;
    }

    /**
     * Remove stale entries and add the new one.
     */
    public void put(double timeS, T value) {
        if (DEBUG) {
            System.out.printf("put %f %s\n", timeS, value);
        }
        while (!m_pastSnapshots.isEmpty()) {
            Entry<Double, T> oldest = m_pastSnapshots.firstEntry();
            Double oldestTimeS = oldest.getKey();
            double oldestAgeS = timeS - oldestTimeS;
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
    public void reset(double timeS, T value) {
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
    public T get(double timeSeconds) {
        // Special case for when the requested time is the same as a sample
        T nowEntry = m_pastSnapshots.get(timeSeconds);
        if (nowEntry != null) {
            if (DEBUG) {
                System.out.printf("record for now %.2f\n", timeSeconds);
            }
            return nowEntry;
        }
        Entry<Double, T> topBound = m_pastSnapshots.ceilingEntry(timeSeconds);
        Entry<Double, T> bottomBound = m_pastSnapshots.floorEntry(timeSeconds);

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

        double timeSinceBottom = timeSeconds - bottomBound.getKey();
        double timeSpan = topBound.getKey() - bottomBound.getKey();
        double timeFraction = timeSinceBottom / timeSpan;
        if (DEBUG) {
            System.out.printf("timeSeconds %f bottom %f top %f\n",
                    timeSeconds, bottomBound.getKey(), topBound.getKey());

            System.out.printf("interpolate %f\n", timeFraction);
        }
        return bottomBound.getValue().interpolate(
                topBound.getValue(), timeFraction);
    }

    /** Items after the given timestamp, optionally inclusive */
    public SortedMap<Double, T> tailMap(double t, boolean inclusive) {
        return m_pastSnapshots.tailMap(t, inclusive);
    }

    /** True if the timestamp is older than the history window. */
    public boolean tooOld(double timestampS) {
        Double newestSeenS = m_pastSnapshots.lastKey();
        double oldestAcceptableS = newestSeenS - m_historyS;
        return timestampS < oldestAcceptableS;
    }

    /** Return entry for key strictly smaller than t. */
    public Entry<Double, T> lowerEntry(double t) {
        return m_pastSnapshots.lowerEntry(t);
    }

    /** Return entry for key strictly greater than t */
    public Entry<Double, T> ceilingEntry(double t) {
        return m_pastSnapshots.ceilingEntry(t);
    }

    public int size() {
        return m_pastSnapshots.size();
    }

    /** Key of the most-recent snapshot. */
    public double lastKey() {
        return m_pastSnapshots.lastKey();
    }

    public Set<Map.Entry<Double, T>> entrySet() {
        return m_pastSnapshots.entrySet();
    }

    public Map<Double, T> all() {
        return m_pastSnapshots;
    }

    /** Print the entire buffer */
    public void dump() {
        for (var x : entrySet()) {
            System.out.printf("%s: %s\n", x.getKey(), x.getValue());
        }
    }
}