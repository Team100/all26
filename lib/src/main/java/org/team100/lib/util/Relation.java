package org.team100.lib.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleFunction;
import java.util.function.ToDoubleFunction;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/** A relation is a set of double-valued tuples. */
public class Relation<V> {
    private final List<V> m_tuples;

    public Relation() {
        m_tuples = new ArrayList<>();
    }

    public void add(V v) {
        m_tuples.add(v);
    }

    public void addAll(List<V> l) {
        m_tuples.addAll(l);
    }

    public List<V> tuples() {
        return m_tuples;
    }

    /**
     * Construct a map with the specified key, and the tuple as the value.
     * 
     * @param key          extractor
     * @param interpolator for the whole tuple
     */
    public InterpolatingMap100<Double, V> map(
            ToDoubleFunction<V> key,
            Interpolator<V> interpolator) {
        InterpolatingMap100<Double, V> map = new InterpolatingMap100<>(
                InverseInterpolator.forDouble(), interpolator);
        for (V p : m_tuples) {
            map.put(key.applyAsDouble(p), p);
        }
        return map;
    }

    /**
     * A unary function backed by an interpolating map.
     * 
     * Makes no attempt to verify anything about the function, e.g. injectivity or
     * smoothness. Watch out.
     * 
     * This is a double function (not an operator) because it can return null.
     * 
     * @param x extractor
     * @param y extractor
     */
    public DoubleFunction<Double> function(
            ToDoubleFunction<V> x, ToDoubleFunction<V> y) {
        InterpolatingMap100<Double, Double> map = new InterpolatingMap100<>(
                InverseInterpolator.forDouble(), Interpolator.forDouble());
        for (V p : m_tuples) {
            map.put(x.applyAsDouble(p), y.applyAsDouble(p));
        }
        return map::get;
    }
}
