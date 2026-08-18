package org.team100.lib.spline.rn;

import org.team100.lib.geometry.rn.WaypointRn;
import org.team100.lib.spline.r1.SplineR1;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

/**
 * An N-dimensional spline, made of N individual SplineR1's.
 * 
 * TODO: also dx, ddx, etc.
 */
public class SplineRn<N extends Num> {
    private static final boolean DEBUG = false;

    private final Nat<N> m_dim;
    private final SplineR1 m_splines[];

    public SplineRn(Nat<N> dim, WaypointRn<N> p0, WaypointRn<N> p1) {
        m_dim = dim;
        m_splines = new SplineR1[dim.getNum()];
        // start and end second derivatives are always zero,
        // i.e. zero jerk at the ends.
        // Note, maybe zero jerk isn't that important?
        // TODO: use dx as ddx instead, since it's quicker?
        double ddx0 = 0;
        double ddx1 = 0;
        for (int i = 0; i < dim.getNum(); ++i) {
            double x0 = p0.position().get(i);
            double x1 = p1.position().get(i);
            double dx0 = p0.velocity().get(i);
            double dx1 = p1.velocity().get(i);
            SplineR1 spline = SplineR1.get(x0, x1, dx0, dx1, ddx0, ddx1);
            if (DEBUG)
                System.out.printf("%d %s\n", i, spline);
            m_splines[i] = spline;
        }
    }

    /** Sample the splines. */
    public Vector<N> x(double s) {
        Vector<N> x = new Vector<>(m_dim);
        for (int i = 0; i < m_dim.getNum(); ++i) {
            x.set(i, 0, m_splines[i].getPosition(s));
        }
        return x;
    }

}
