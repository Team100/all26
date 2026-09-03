package org.team100.lib.util;

import java.util.Random;

import org.junit.jupiter.api.Test;
import org.wpilib.math.filter.LinearFilter;

public class LowPassDerivativeTest {
    private static final boolean DEBUG = false;

    @Test
    void test0() {
        // backwards finite difference alone
        double dt = 0.02;
        // signal freq is 2 hz
        double freqHz = 2;
        double stdDev = 0.1;
        LinearFilter f1 = LinearFilter.backwardFiniteDifference(1, 2, dt);
        LinearFilter f2 = LinearFilter.backwardFiniteDifference(1, 2, dt);
        Random rand = new Random(100);
        for (double t = 0; t < 5; t += dt) {
            double cleanInput = Math.sin(freqHz * 2 * Math.PI * t);
            // noise frequency is 25 hz (1/2 sample rate)
            double noise = rand.nextGaussian(0, stdDev);
            double noisyInput = cleanInput + noise;
            double cleanOutput = f1.calculate(cleanInput);
            double noisyOutput = f2.calculate(noisyInput);
            double error = cleanOutput - noisyOutput;
            if (DEBUG)
                System.out.printf("%8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f\n",
                        t, cleanInput, noise, noisyInput, cleanOutput, noisyOutput, error);
        }
    }

    @Test
    void test1() {
        // IIR low-pass filter and then backward finite difference
        // sample freq is 50 hz.
        double dt = 0.02;
        // signal freq is 2 hz.
        double freqHz = 2;
        double stdDev = 0.1;
        // filter cutoff is twice the signal freq
        double T = 1 / (2 * Math.PI * 2 * freqHz);
        // System.out.printf("T %f\n", T);
        LinearFilter f0 = LinearFilter.singlePoleIIR(T, dt);
        LinearFilter f1 = LinearFilter.backwardFiniteDifference(1, 2, dt);
        LinearFilter f2 = LinearFilter.backwardFiniteDifference(1, 2, dt);
        Random rand = new Random(100);
        for (double t = 0; t < 5; t += dt) {
            double cleanInput = Math.sin(freqHz * 2 * Math.PI * t);
            // noise frequency is 25 hz (1/2 sample rate)
            double noise = rand.nextGaussian(0, stdDev);
            double noisyInput = cleanInput + noise;
            double filteredInput = f0.calculate(noisyInput);
            double cleanOutput = f1.calculate(cleanInput);
            double filteredOutput = f2.calculate(filteredInput);
            double error = cleanOutput - filteredOutput;
            if (DEBUG)
                System.out.printf("%8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f\n",
                        t, cleanInput, noise, noisyInput, cleanOutput, filteredOutput, error);
        }
    }

    @Test
    void test2() {
        // moving-average filter and then backward finite difference
        // sample freq is 50 hz.
        double dt = 0.02;
        // signal freq is 2 hz.
        double freqHz = 2;
        double stdDev = 0.1;
        LinearFilter f0 = LinearFilter.movingAverage(4);
        LinearFilter f1 = LinearFilter.backwardFiniteDifference(1, 2, dt);
        LinearFilter f2 = LinearFilter.backwardFiniteDifference(1, 2, dt);
        Random rand = new Random(100);
        for (double t = 0; t < 5; t += dt) {
            double cleanInput = Math.sin(freqHz * 2 * Math.PI * t);
            // noise frequency is 25 hz (1/2 sample rate)
            double noise = rand.nextGaussian(0, stdDev);
            double noisyInput = cleanInput + noise;
            double filteredInput = f0.calculate(noisyInput);
            double cleanOutput = f1.calculate(cleanInput);
            double filteredOutput = f2.calculate(filteredInput);
            double error = cleanOutput - filteredOutput;
            if (DEBUG)
                System.out.printf("%8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f\n",
                        t, cleanInput, noise, noisyInput, cleanOutput, filteredOutput, error);
        }
    }

    @Test
    void test3() {
        // sinc gains produce rectangular passband, or would, if the gains
        // were symmetric (noncausal) and infinite in extent. :-)
        double[] sincGains = sincGains(0.125, 4);
        // System.out.println(StrUtil.arrayStr(sincGains));
        double dt = 0.02;
        // signal freq is 2 hz.
        double freqHz = 2;
        double stdDev = 0.1;
        LinearFilter f0 = new LinearFilter(sincGains, new double[0]);
        LinearFilter f1 = LinearFilter.backwardFiniteDifference(1, 2, dt);
        LinearFilter f2 = LinearFilter.backwardFiniteDifference(1, 2, dt);
        Random rand = new Random(100);
        for (double t = 0; t < 5; t += dt) {
            double cleanInput = Math.sin(freqHz * 2 * Math.PI * t);
            // noise frequency is 25 hz (1/2 sample rate)
            double noise = rand.nextGaussian(0, stdDev);
            double noisyInput = cleanInput + noise;
            double filteredInput = f0.calculate(noisyInput);
            double cleanOutput = f1.calculate(cleanInput);
            double filteredOutput = f2.calculate(filteredInput);
            double error = cleanOutput - filteredOutput;
            if (DEBUG)
                System.out.printf("%8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f\n",
                        t, cleanInput, noise, noisyInput, cleanOutput, filteredOutput, error);
        }
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
