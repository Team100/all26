package org.team100.lib.util;

import java.util.TreeMap;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Like InterpolatingTreeMap, but returns null for off-the-edge samples.
 */
public class InterpolatingMap100<K, V> {
    private final TreeMap<K, V> m_map;
    private final InverseInterpolator<K> m_inverseInterpolator;
    private final Interpolator<V> m_interpolator;

    public InterpolatingMap100(
            InverseInterpolator<K> inverseInterpolator,
            Interpolator<V> interpolator) {
        m_map = new TreeMap<>();
        m_inverseInterpolator = inverseInterpolator;
        m_interpolator = interpolator;
    }

    public static InterpolatingMap100<Double, Double> forDouble() {
        return new InterpolatingMap100<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    }

    public void put(K key, V value) {
        m_map.put(key, value);
    }

    public K firstKey() {
        return m_map.firstKey();
    }

    public K lastKey() {
        return m_map.lastKey();
    }

    /** Returns null for off-the-end keys. */
    public V get(K key) {
        V val = m_map.get(key);
        if (val != null) {
            return val;
        }

        K ceilingKey = m_map.ceilingKey(key);
        K floorKey = m_map.floorKey(key);

        if (ceilingKey == null || floorKey == null) {
            return null;
        }
        V floor = m_map.get(floorKey);
        V ceiling = m_map.get(ceilingKey);

        return m_interpolator.interpolate(
                floor,
                ceiling,
                m_inverseInterpolator.inverseInterpolate(
                        floorKey, ceilingKey, key));

    }

    public double size() {
        return m_map.size();
    }
}
