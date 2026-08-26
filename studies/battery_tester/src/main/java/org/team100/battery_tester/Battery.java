package org.team100.battery_tester;

/** Battery base class. */
public abstract class Battery {
    private static final boolean DEBUG = false;

    /** Operating point of the battery. */
    public record Op(double v, double i) {
    }

    /** Open-circuit voltage, a positive number */
    abstract double V0();

    /** Resistance, ohms. */
    abstract double R();

    /** State of charge, [0, 1] */
    abstract double SOC();

    /**
     * Computes "sag", the voltage at which the desired current can be delivered.
     * 
     * Applies Ohm's and Kirchoff's laws:
     * 
     * V(I) = V0 - I*R
     * 
     * Never returns a negative number; might return zero.
     */
    public double V(double i) {
        if (i < 0)
            throw new IllegalArgumentException();
        double v0 = V0();
        double r = R();
        // avoid zero
        double v = Math.max(1e-6, v0 - i * r);
        if (DEBUG)
            System.out.printf("BatteryBase: VforI success v0 %f r %f v %f i %f soc %f\n",
                    v0, r, v, i, SOC());
        return v;
    }

    /** Operating point of the battery for the given power (watts). */
    public Op operatingPoint(double p) {
        double inputV = V0();
        double inputI = p / inputV;

        for (int j = 0; j < 1000; ++j) {
            // New voltage estimate.
            inputI = p / inputV;
            inputV = V(inputI);
            double p0 = inputV * inputI;
            if (Math.abs(p - p0) < 0.001) {
                // System.out.printf("solved %d\n", j);
                return new Op(inputV, inputI);
            }
        }
        System.out.printf("Battery.operatingPoint: fail");
        return new Op(inputV, inputI);

    }
}
