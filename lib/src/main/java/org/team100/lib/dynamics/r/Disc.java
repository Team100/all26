package org.team100.lib.dynamics.r;

/**
 * The simplest R dynamics, for a balanced disc. It has only inertia,
 * so the dynamics are:
 * 
 * t = I * alpha
 * 
 * https://en.wikipedia.org/wiki/List_of_moments_of_inertia
 */
public class Disc implements RDynamics {
    /** Moment of inertia, kg m^2 */
    private final double izz1;

    /**
     * @param I inertia kg m^2
     */
    public Disc(double I) {
        izz1 = I;
    }

    /**
     * Uniform disc.
     * 
     * https://en.wikipedia.org/wiki/List_of_moments_of_inertia
     * 
     * @param M mass kg
     * @param R radius m
     */
    public static Disc uniform(double M, double R) {
        return new Disc(0.5 * M * R * R);
    }

    @Override
    public REffort effort(RConfig q, RVelocity qdot, RAcceleration qddot) {
        // t = I * alpha
        return new REffort(izz1 * qddot.q1ddot());
    }

    @Override
    public RAcceleration qddot(RConfig q, RVelocity qdot, REffort effort) {
        // alpha = t / I
        return new RAcceleration(effort.t() / izz1);
    }

}
