package org.team100.lib.dynamics.r;

/**
 * Gravity is in the -x direction.
 */
public class RDynamicsAnalytic implements RDynamics {
    /** Gravity, m/s^2. */
    private static final double g = 9.8;
    /** Mass, kg. */
    private final double m1;
    /** Length of link, m. */
    @SuppressWarnings("unused")
    private final double l1;
    /** Distance from q to the link center of mass, m. */
    private final double lc1;
    /** Moment of inertia, kg m^2. */
    private final double izz1;

    /** Arm. */
    public RDynamicsAnalytic(double m1, double l1, double lc1, double izz1) {
        this.m1 = m1;
        this.l1 = l1;
        this.lc1 = lc1;
        this.izz1 = izz1;
    }

    /**
     * Thin rod, center of mass in the geometric center.
     * 
     * https://en.wikipedia.org/wiki/List_of_moments_of_inertia
     * 
     * @param M mass in kg
     * @param L length in m
     */
    public static RDynamicsAnalytic thinRod(double M, double L) {
        return new RDynamicsAnalytic(M, L, L / 2, M * L * L / 12);
    }

    /**
     * Generalized force (torque or force) to achieve the required
     * acceleration, and also to oppose gravity.
     * 
     * Note: R dynamics don't actually depend on velocity.
     */
    @Override
    public REffort effort(RConfig q, RVelocity qdot, RAcceleration qddot) {
        double s1 = Math.sin(q.q1());
        double m11 = m1 * lc1 * lc1 + izz1;
        double g1 = -m1 * g * lc1 * s1;
        double t1 = m11 * qddot.q1ddot() + g1;
        return new REffort(t1);
    }

    @Override
    public RAcceleration qddot(RConfig q, RVelocity qdot, REffort effort) {
        throw new UnsupportedOperationException();
    }

}
