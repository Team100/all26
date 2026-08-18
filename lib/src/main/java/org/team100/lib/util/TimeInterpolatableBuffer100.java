package org.team100.lib.util;

import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.SortedMap;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import edu.wpi.first.math.interpolation.Interpolator;

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
public class TimeInterpolatableBuffer100<T> {
    private static final boolean DEBUG = false;

    private final Interpolator<T> m_interpolator;
    /** Length of the buffer in seconds */
    private final double m_historyS;
    /** Key is timestamp in seconds */
    private final NavigableMap<Double, T> m_pastSnapshots = new ConcurrentSkipListMap<>();

    /**
     * We allow concurrent operations on the map, with one exception: when we want
     * two reads to be consistent with each other. For this purpose we use a
     * read-write lock with the semantics inverted: it's the double-read operation
     * that takes the "write" (exclusive) lock, and the write operations take the
     * "read" (non-exclusive) lock.
     */
    private final ReadWriteLock m_lock = new ReentrantReadWriteLock();

    public TimeInterpolatableBuffer100(
            Interpolator<T> interpolator,
            double historyS,
            double timeS,
            T initialValue) {
        m_interpolator = interpolator;
        m_historyS = historyS;
        m_pastSnapshots.put(timeS, initialValue);
    }

    /**
     * Remove stale entries and add the new one.
     */
    public void put(double timeS, T value) {
        if (DEBUG) {
            System.out.printf("put %f %s\n", timeS, value);
        }
        try {
            // wait for in-progress double-reads
            m_lock.readLock().lock();
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
        } finally {
            m_lock.readLock().unlock();
        }
    }

    /**
     * Remove all entries and add the new one.
     */
    public void reset(double timeS, T value) {
        if (DEBUG) {
            System.out.printf("reset %f %s\n", timeS, value);
        }
        try {
            // wait for in-progress double-reads
            m_lock.readLock().lock();
            m_pastSnapshots.clear();
            m_pastSnapshots.put(timeS, value);
        } finally {
            m_lock.readLock().unlock();
        }
    }

    /**
     * Sample the buffer at the given time.
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
        Entry<Double, T> topBound = null;
        Entry<Double, T> bottomBound = null;
        try {
            // this pair should be consistent, so prohibit writes briefly.
            m_lock.writeLock().lock();
            topBound = m_pastSnapshots.ceilingEntry(timeSeconds);
            bottomBound = m_pastSnapshots.floorEntry(timeSeconds);
        } finally {
            m_lock.writeLock().unlock();
        }
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
        return m_interpolator.interpolate(
                bottomBound.getValue(), topBound.getValue(), timeFraction);
    }

    public SortedMap<Double, T> tailMap(double t, boolean inclusive) {
        return m_pastSnapshots.tailMap(t, inclusive);
    }

    /** True if the timestamp is older than the history window. */
    public boolean tooOld(double timestampS) {
        Double newestSeenS = m_pastSnapshots.lastKey();
        double oldestAcceptableS = newestSeenS - m_historyS;
        return timestampS < oldestAcceptableS;
    }

    public Entry<Double, T> lowerEntry(double t) {
        return m_pastSnapshots.lowerEntry(t);
    }

    public Entry<Double, T> ceilingEntry(double arg0) {
        return m_pastSnapshots.ceilingEntry(arg0);
    }

    public int size() {
        return m_pastSnapshots.size();
    }

    /** Timestamp of the most-recent snapshot. */
    public double lastKey() {
        return m_pastSnapshots.lastKey();
    }

    /** Print the entire buffer */
    public void dump() {
        for (var x : m_pastSnapshots.entrySet()) {
            System.out.printf("%s: %s\n", x.getKey(), x.getValue());
        }
    }
}