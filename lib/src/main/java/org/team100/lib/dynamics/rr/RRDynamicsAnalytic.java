package org.team100.lib.dynamics.rr;

import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRVelocity;

public class RRDynamicsAnalytic implements RRDynamics {
    /** Gravity */
    private static final double g = 9.8;
    /** Mass of the moving elevator parts. */
    private final double m1;
    /** Mass of the arm. */
    private final double m2;
    /** Length of link 1 */
    private final double l1;
    /** Length of link 2 */
    @SuppressWarnings("unused")
    private final double l2;
    /** Distance from q1 to the link 1 center of mass. */
    private final double lc1;
    /** Distance from q2 to the link 2 center of mass. */
    private final double lc2;
    /** Moment of inertia of link 1. */
    private final double izz1;
    /** Moment of inertia of link 1. */
    private final double izz2;

    public RRDynamicsAnalytic(
            double m1, double m2, double l1, double l2,
            double lc1, double lc2, double izz1, double izz2) {
        this.m1 = m1;
        this.m2 = m2;
        this.l1 = l1;
        this.l2 = l2;
        this.lc1 = lc1;
        this.lc2 = lc2;
        this.izz1 = izz1;
        this.izz2 = izz2;
    }

    /**
     * Thin rods, center of mass in the geometric center.
     * 
     * https://en.wikipedia.org/wiki/List_of_moments_of_inertia
     * 
     * @param M1 mass in kg
     * @param M2 mass in kg
     * @param L1 length in m
     * @param L2 length in m
     */
    public static RRDynamicsAnalytic thinRod(
            double M1, double M2, double L1, double L2) {
        return new RRDynamicsAnalytic(
                M1, M2,
                L1, L2,
                L1 / 2, L2 / 2,
                M1 * L1 * L1 / 12, M2 * L2 * L2 / 12);
    }

    /**
     * Generalized force (torque or force) to achieve the required
     * velocity and acceleration, and also to oppose gravity.
     */
    public RREffort effort(RRConfig q, RRVelocity v, RRAcceleration a) {
        double s1 = Math.sin(q.q1());
        double s2 = Math.sin(q.q2());
        double c2 = Math.cos(q.q2());
        double s12 = Math.sin(q.q1() + q.q2());

        double m11 = m1 * lc1 * lc1 + m2 * l1 * l1
                + 2 * m2 * l1 * lc2 * c2 + m2 * lc2 * lc2
                + izz1 + izz2;
        double m12 = m2 * l1 * lc2 * c2 + m2 * lc2 * lc2 + izz2;
        double m21 = m2 * l1 * lc2 * c2 + m2 * lc2 * lc2 + izz2;
        double m22 = m2 * lc2 * lc2 + izz2;

        double c11 = -m2 * l1 * lc2 * s2 * v.q2dot();
        double c12 = -m2 * l1 * lc2 * s2 * v.q1dot() - m2 * l1 * lc2 * s2 * v.q2dot();
        double c21 = m2 * l1 * lc2 * s2 * v.q1dot();
        double c22 = 0;

        double g1 = -m1 * g * lc1 * s1
                - m2 * g * (lc2 * s12 + l1 * s1);
        double g2 = -m2 * g * lc2 * s12;

        double t1 = m11 * a.q1ddot()
                + m12 * a.q2ddot()
                + c11 * v.q1dot()
                + c12 * v.q2dot()
                + g1;

        double t2 = m21 * a.q1ddot()
                + m22 * a.q2ddot()
                + c21 * v.q1dot()
                + c22 * v.q2dot()
                + g2;

        return new RREffort(t1, t2);
    }

    @Override
    public RRAcceleration qddot(RRConfig q, RRVelocity qdot, RREffort effort) {
        throw new UnsupportedOperationException();
    }

}
