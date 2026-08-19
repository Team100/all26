package org.team100.lib.util;

public class Clamp {
    private final double min;
    private final double max;

    public Clamp(double min, double max) {
        this.min = min;
        this.max = max;
    }

    public double f(double x) {
        return Math.clamp(x, min, max);
    }

}
