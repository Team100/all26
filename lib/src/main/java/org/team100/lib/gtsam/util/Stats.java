package org.team100.lib.gtsam.util;

/**
 * 
 * https://nestedsoftware.com/2018/03/27/calculating-standard-deviation-on-streaming-data-253l.23919.html
 * 
 */
public class Stats {

    private int count = 0;
    private double _mean = 0;
    private double _dSquared = 0;

    /**
     * For when the measurement is in nanoseconds, this prints mean and stddev in ms
     */
    public void nanoDump() {
        System.out.printf(
                "ET (ms) mean %7.5f stddev %7.5f\n",
                mean() / 1000000,
                populationStdev() / 1000000);
    }

    /** The raw mean and stddev */
    public void dump() {
        System.out.printf(
                "mean  %7.5f stddev %7.5f\n",
                mean(),
                populationStdev());
    }

    public void update(double newValue) {
        count++;
        double meanDifferential = (newValue - _mean) / count;
        double newMean = _mean + meanDifferential;
        double dSquaredIncrement = (newValue - newMean) * (newValue - _mean);
        double newDSquared = this._dSquared + dSquaredIncrement;
        _mean = newMean;
        _dSquared = newDSquared;
    }

    public void reset() {
        count = 0;
        _mean = 0;
        _dSquared = 0;
    }

    public double mean() {
        validate();
        return _mean;
    }

    double dSquared() {
        validate();
        return _dSquared;
    }

    double populationVariance() {
        return _dSquared / count;
    }

    double populationStdev() {
        return Math.sqrt(populationVariance());
    }

    double sampleVariance() {
        return count > 1 ? _dSquared / (count - 1) : 0;
    }

    double sampleStdev() {
        return Math.sqrt(sampleVariance());
    }

    void validate() {
        if (this.count == 0) {
            throw new IllegalStateException("Mean is undefined");
        }
    }
}
