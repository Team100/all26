package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import java.util.Collection;
import java.util.function.BiPredicate;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

public class CoaslescingCollectionTest {
    @Test
    void test1d() {
        TrailingHistory<Double> history = new TrailingHistory<>();
        BiPredicate<Double, Double> near = new BiPredicate<>() {
            @Override
            public boolean test(Double a, Double b) {
                return Math.abs(a - b) < 1;
            }
        };
        Function<Collection<Double>, Double> combine = new Function<>() {

            @Override
            public Double apply(Collection<Double> t) {
                return t.stream().collect(Collectors.averagingDouble(x -> x));
            }

        };
        CoalescingCollection<Double> c = new CoalescingCollection<>(
                history, near, combine);
        c.evict(-3);
        c.add(0.0, 10.0);
        assertArrayEquals(
                new double[] { 10.0 },
                c.getAll().stream().mapToDouble(Double::doubleValue).toArray());
        c.evict(-2);
        c.add(1.0, 20.0);
        assertArrayEquals(
                new double[] { 10.0, 20.0 },
                c.getAll().stream().mapToDouble(Double::doubleValue).toArray());
        // near 10
        c.evict(-1);
        c.add(2.0, 10.5);
        assertArrayEquals(
                new double[] { 20.0, 10.25 },
                c.getAll().stream().mapToDouble(Double::doubleValue).toArray());
        // combines with the near one, and also expires the other one
        c.evict(0);
        c.add(3.0, 10.5);
        assertArrayEquals(
                new double[] { 20.0, 10.375 },
                c.getAll().stream().mapToDouble(Double::doubleValue).toArray());
    }

}
