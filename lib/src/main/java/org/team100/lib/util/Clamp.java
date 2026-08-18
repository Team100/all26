package org.team100.lib.util;

import edu.wpi.first.math.MathUtil;

public class Clamp {
    private final double min;
    private final double max;

    public Clamp(double min, double max) {
        this.min = min;
        this.max = max;
    }

    public double f(double x) {
        return MathUtil.clamp(x, min, max);
    }

}
