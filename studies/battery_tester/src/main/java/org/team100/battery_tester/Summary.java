package org.team100.battery_tester;

/** Summary of a batch of measurements, used only by Summarizer. */
class Summary {
    /** First timestamp received. */
    private Double t0;
    /** Number of points in the batch */
    int n;
    /** Time since t0 */
    double tMax;
    double iSum;
    double vSum;
    double pSum;
    double qSum;

    /** For trapezoid integration. */
    private double iPrev;

    public Summary(Summary prev) {
        if (prev != null) {
            t0 = prev.t0;
            qSum = prev.qSum;
            tMax = prev.tMax;
            iPrev = prev.iPrev;
        }
    }

    /**
     * Add a measurement.
     * 
     * @param t timestamp, seconds, monotonic!
     * @param i current, amperes
     * @param v voltage, volts
     * @param p power, watts
     */
    public void add(double t, double i, double v, double p) {
        n += 1;
        iSum += i;
        vSum += v;
        pSum += p;
        if (t0 == null) {
            // First item of the whole run.
            t0 = t;
            tMax = 0.0;
        }
        double dq = getDq(t, i);
        qSum += dq;
    }

    /** Time since previous measurement. */
    private double getDt(double timestamp) {
        // Time since the start of the run.
        double ET = timestamp - t0;
        // Time since the previous measurement.
        double dt = ET - tMax;
        tMax = ET;
        return dt;
    }

    /** Charge since previous measurement, trapezoid integration */
    private double getDq(double t, double i) {
        double iAvg = (iPrev + i) / 2;
        double dt = getDt(t);
        double dq = iAvg * dt;
        iPrev = i;
        return dq;
    }

    public String toString() {
        return String.format("%10.3f, %10.3f, %10.3f, %10.3f, %10.3f",
                tMax, iSum / n, vSum / n, pSum / n, qSum);
    }
}