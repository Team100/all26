package org.team100.lib.dynamics.p;

/** Dynamics for prismatic (linear) joint. The only parameter is mass. */
public class PDynamics {
    private final double m;

    public PDynamics(double m) {
        this.m = m;
    }

    /**
     * Force to achieve the required acceleration.
     * 
     * Note there is no config or velocity here because
     * the dynamics don't depend on them.
     */
    public PEffort effort(PAcceleration a) {
        double F = m * a.qddot();
        return new PEffort(F);
    }

    /**
     * We often model a shooter drum or intake roller as a linear
     * mechanism, because the quantity of interest is the surface
     * speed.
     * 
     * This method "translates" the drum inertia into the equivalent
     * "mass" for the prismatic dynamics.
     * 
     * @param I inertia in kg m^2
     * @param r radius in m
     */
    public static PDynamics drum(double I, double r) {
        // the actual dynamics are t = I alpha
        //
        // t = r * F
        //
        // a = r * alpha => alpha = a/r
        //
        // thus F = (I/r^2) * a
        return new PDynamics(I / (r * r));
    }

}
