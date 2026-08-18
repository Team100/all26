package org.team100.lib.uncertainty;

import edu.wpi.first.math.MathUtil;

/** Uncertain variable in one dimension. */
public class VariableR1 {
    private final double mean;
    private final double variance;

    private VariableR1(double mean, double variance) {
        this.mean = mean;
        this.variance = variance;
    }

    public static VariableR1 fromStdDev(double mean, double sigma) {
        return new VariableR1(mean, sigma * sigma);
    }

    public static VariableR1 fromVariance(double mean, double variance) {
        return new VariableR1(mean, variance);
    }

    /**
     * Linear interpolation of mean and variance.
     * 
     * mean = t * this + (1-t) * other
     * variance = t^2 * this + (1-t)^2 * other + 2*t(1-t) * cov
     * 
     * if we assume that the operands are completely correlated, then this is
     * 
     * variance = (t^2 + 1 - 2t + t^2 + 2t - 2t^2)*var
     *
     * which means the variance is the same as the operand variance.
     */
    public VariableR1 interpolate(VariableR1 other, double t) {
        return VariableR1.fromVariance(
                MathUtil.interpolate(mean, other.mean, t),
                MathUtil.interpolate(variance, other.variance, t));
    }

    /**
     * Operands are independent, so means and variances simply add.
     */
    public static VariableR1 add(VariableR1 a, VariableR1 b) {
        return VariableR1.fromVariance(a.mean + b.mean, a.variance + b.variance);
    }

    /** Return a-b with additive variance. */
    public static VariableR1 subtract(VariableR1 a, VariableR1 b) {
        return VariableR1.fromVariance(a.mean - b.mean, a.variance + b.variance);
    }

    /**
     * Multiply this variable by x.
     * 
     * Variance is multiplied by x^2, i.e. standard deviation is affected linearly.
     */
    public VariableR1 times(double x) {
        return new VariableR1(x * mean(), x * x * variance());
    }

    public double mean() {
        return mean;
    }

    public double variance() {
        return variance;
    }

    /** For testing */
    public double sigma() {
        return Math.sqrt(variance);
    }

    @Override
    public String toString() {
        return String.format(
                "[mean=%14.9e sigma=%14.9e]",
                mean(), sigma());
    }

}
