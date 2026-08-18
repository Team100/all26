package org.team100.lib.optimization;

import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.MathUtil;

/**
 * 1d specialization that avoids vectors.
 */
public class NewtonsMethod1d {
    private final DoubleUnaryOperator m_f;
    private final double m_xMin;
    private final double m_xMax;
    private final double m_tolerance;
    private final int m_iterations;
    private final double m_dxLimit;

    public NewtonsMethod1d(
            DoubleUnaryOperator f,
            double xMin,
            double xMax,
            double tolerance,
            int iterations,
            double dxLimit) {
        m_f = f;
        m_xMin = xMin;
        m_xMax = xMax;
        m_tolerance = tolerance;
        m_iterations = iterations;
        m_dxLimit = dxLimit;

    }

    /** Mirrors NewtonsMethod.solve() */
    public double solve(double initial) {
        double x = initial;
        for (int i = 0; i < m_iterations; ++i) {
            double error = m_f.applyAsDouble(x);
            if (within(error)) {
                return x;
            }
            double j = NumericalJacobian100.numericalJacobian1d(m_f, x);
            if (j > 1e6) {
                // avoid division by anything close to zero
                return x;
            }
            double dx = error / j;
            // Too-high dx results in oscillation.
            dx = clamp(dx);
            x = update(x, dx);
            // Keep the x estimate within bounds.
            x = limit(x);
        }
        return x;
    }

    private boolean within(double error) {
        return Math.abs(error) < m_tolerance;
    }

    /**
     * Limit: dx = clamp(dx) using a fixed limit.
     */
    private double clamp(double dx) {
        return MathUtil.clamp(dx, -m_dxLimit, m_dxLimit);
    }

    /**
     * Update: x = x - dx.
     * Note the minus sign: negative slope means x should move to the right.
     */
    private double update(double x, double dx) {
        return x - dx;
    }

    /**
     * Clamp the guess to the limits.
     */
    private double limit(double x) {
        return MathUtil.clamp(x, m_xMin, m_xMax);
    }

}
