package org.team100.cartpole;

import org.jfree.data.xy.VectorSeries;
import org.team100.math.Simulator;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N4;

/** Visualize the pole positions. */
public class CartPoleChart {
    public static VectorSeries make(double l, Simulator<N4, N1> sim) {
        int step = 10;
        VectorSeries s = new VectorSeries("sim");
        double[] xSeries = sim.series(0);
        double[] thetaSeries = sim.series(2);
        for (int i = 0; i < xSeries.length; i += step) {
            double x = xSeries[i];
            double theta = thetaSeries[i];
            double dx = l * Math.sin(theta);
            double dy = l * Math.cos(theta);
            s.add(x, 0, dx, dy);
        }
        return s;
    }
}
