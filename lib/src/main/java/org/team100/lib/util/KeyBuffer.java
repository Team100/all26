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

    private final NavigableSet<Double> m_timestamps;

    public KeyBuffer() {
        m_timestamps = new TreeSet<>();
    }

}
