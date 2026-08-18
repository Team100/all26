package org.team100.lib.optimization;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;

import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

/**
 * Line search over each coordinate.
 * 
 * TODO: tolerance and iteration are used for two purposes here,
 * the outer loop and the inner loop for each dimension. Split them.
 * 
 * @see https://en.wikipedia.org/wiki/Coordinate_descent
 */
@SuppressWarnings("unused")
public class CoordinateDescent<R extends Num> {
    private static final int DEBUG = 0;
    private final Nat<R> m_rows;
    private final Function<Vector<R>, Double> m_f;
    private final double m_tolerance;
    private final int m_iterations;

    public CoordinateDescent(
            Nat<R> rows,
            Function<Vector<R>, Double> f,
            double tolerance,
            int iterations) {
        m_rows = rows;
        m_f = f;
        m_tolerance = tolerance;
        m_iterations = iterations;
    }

    public Vector<R> solve(final Vector<R> bottom, final Vector<R> initial, final Vector<R> top) {
        Vector<R> current = initial;
        for (int i = 0; i < m_iterations; ++i) {
            Vector<R> next = new Vector<>(current.getStorage().copy());
            if (DEBUG > 0)
                System.out.printf("i %d x %s\n", i, StrUtil.vecStr(next));
            // System.out.printf("%5.3f\n", next.get(0));
            for (int j = 0; j < m_rows.getNum(); ++j) {
                final int jj = j;
                if (DEBUG > 1)
                    System.out.printf("j %d\n", j);
                DoubleUnaryOperator cf = (x) -> {
                    Vector<R> v = new Vector<>(next.getStorage().copy());
                    v.set(jj, 0, x);
                    double err = m_f.apply(v);
                    if (DEBUG > 1)
                        System.out.printf("err %f\n", err);
                    return err;
                };
                GoldenSectionSearch s = new GoldenSectionSearch(cf, m_tolerance, m_iterations);
                double x = s.solve(bottom.get(j), top.get(j));
                next.set(jj, 0, x);
            }
            double step = next.minus(current).norm();
            if (step < m_tolerance) {
                // System.out.println(i);
                return next;
            }
            current = next;
        }
        return current;
    }

}
