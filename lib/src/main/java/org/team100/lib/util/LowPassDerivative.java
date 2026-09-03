package org.team100.lib.util;

import org.team100.lib.framework.TimedRobot100;
import org.wpilib.math.filter.LinearFilter;

/**
 * We often need to find the derivative of some signal, e.g. given a series of
 * positional setpoints, derive the velocity for motor feedforward control.
 * 
 * The raw backwards finite difference amplifies the high-frequency noise in the
 * signal, causing lots of noise and effort in the motor.
 * 
 * This class combines a causal derivative with a causal low-pass filter, to
 * reduce the high-frequency noise.
 * 
 * NOTE! All causal low-pass filters introduce DELAY, because they necessarily
 * allow past measurements to affect the current one. You may want to experiment
 * with different combinations of cutoff frequency and delay, to see what works
 * best in your application.
 * 
 * There is a comparison of filtering approaches here:
 * 
 * https://docs.google.com/document/d/1M5b7rZ5uNSPQv2hXfvNHeeBB0trK4iKQUhVR-a8g2k8
 */
@SuppressWarnings("unused")
public class LowPassDerivative {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;
    private final LinearFilter f0;
    private final LinearFilter f1;

    public LowPassDerivative() {
        // f0 = smomoth();
        // f0 = boxcar(4);
        f0 = sinc(4);
        f1 = LinearFilter.backwardFiniteDifference(1, 2, DT);
    }

    public double calculate(double noisyInput) {
        double filteredInput = f0.calculate(noisyInput);
        double filteredOutput = f1.calculate(filteredInput);
        return filteredOutput;
    }

    /** for convenience, when updating separately from reading. */
    public double lastValue() {
        return f1.lastValue();
    }

    /** See https://en.wikipedia.org/wiki/Exponential_smoothing */
    private static LinearFilter smooth() {
        double freqHz = 2;
        // filter cutoff is twice the signal freq
        double T = 1 / (2 * Math.PI * 2 * freqHz);
        return LinearFilter.singlePoleIIR(T, DT);
    }

    /** See https://en.wikipedia.org/wiki/Moving_average */
    private static LinearFilter boxcar(int taps) {
        return LinearFilter.movingAverage(taps);
    }

    /**
     * Sinc gains produce rectangular passband, or would, if the gains
     * were symmetric (noncausal) and infinite in extent. :-)
     * See https://en.wikipedia.org/wiki/Sinc_filter
     */
    private static LinearFilter sinc(int taps) {
        // NOTE: f normalization implicitly involves DT
        double[] sincGains = sincGains(0.125, taps);
        return new LinearFilter(sincGains, new double[0]);
    }

    /**
     * @param f cutoff, normalized
     * @param N taps
     */
    private static double[] sincGains(double f, int N) {
        double alpha = alpha(N);
        double[] gains = new double[N];
        for (int i = 0; i < N; ++i) {
            gains[i] = 2 * f * sinc(2 * f * (i - alpha));
        }
        return gains;
    }

    private static double alpha(double N) {
        return (N - 1) / 2;
    }

    private static double sinc(double x) {
        return Math.sin(Math.PI * x) / (Math.PI * x);
    }

}
