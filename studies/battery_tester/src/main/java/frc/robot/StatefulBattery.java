package frc.robot;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Battery model including charge state.
 * 
 * This model is intended for simulation, with the following refinements:
 * 
 * * the voltage source depends on charge state.
 * 
 * We use the Powersonic PS-12180
 * (https://www.power-sonic.com/product/ps-12180/), which
 * is rated at 18Ah for 20h discharge rate. (This is mandated by rule,
 * "20-hour discharge rate: minimum 17Ah, maximum 18.2Ah")
 * 
 * https://www.scribd.com/document/48734929/A-mathematical-model-for-lead-acid-batteries
 * https://www.mathworks.com/content/dam/mathworks/tag-team/Objects/s/40542_SAE-2007-01-0778-Battery-Modeling-Process.pdf
 * https://ut3-toulouseinp.hal.science/hal-03539078v1/document
 * https://www.athensjournals.gr/technology/2016-3-3-4-Dost.pdf
 * https://en.wikipedia.org/wiki/Peukert's_law
 * https://www.power-sonic.com/product/ps-12180/
 */
public class StatefulBattery extends BatteryBase {
    private static final boolean DEBUG = false;
    /** Open Circuit Voltage as a function of State of Charge. */
    final InterpolatingDoubleTreeMap ocv;
    /** Internal resistance as a function of State of Charge. */
    final InterpolatingDoubleTreeMap r;
    /** Rated capacity at 20h discharge rate, coulombs. */
    final double c0;
    /** 20h discharge rate in amps. */
    final double i0;
    /** Peukert's constant. */
    final double k;
    /** State of charge, coulombs. */
    private double c;

    public StatefulBattery() {
        ocv = makeOCV();
        r = makeR();
        c0 = 18 * 3600;
        // state of charge starts at c0, fully charged.
        c = c0;
        // for testing low-SOC
        // c = c0 / 10;
        // 18 Ah / 20 h = 0.9 A.
        i0 = 18.0 / 20;
        // Adjusted to fit the discharge rates published by PowerSonic.
        // The published rates don't fit Peukert's law very well, and there are only
        // three points to fit.
        // I just used the highest rate since it's closest to the (very high) current we
        // actually use.
        // see
        // https://docs.google.com/spreadsheets/d/1gB8hojtICp1v2dbFhKkuQ7yY3v1sJFDWeo-w5ABYFcA/edit?gid=862420482#gid=862420482
        k = 1.1641;
    }

    /** Discharge at the specified current (amps) for the specified time (sec). */
    public void discharge(double i, double t) {
        if (Double.isInfinite(i) || Double.isNaN(i))
            throw new IllegalArgumentException();
        // derating is less than 1
        double derate = peukert(i);
        // derated (i.e. additional) current
        double i1 = i / derate;

        // derated coulombs produced
        double dc = i1 * t;

        if (DEBUG)
            System.out.printf("StatefulBattery.discharge: i %f derate %f dc %f i1 %f\n",
                    i, derate, dc, i1);
        setC(c - dc);
    }

    @Override
    double SOC() {
        return Math.clamp(c / c0, 0, 1);
    }

    @Override
    double V0() {
        return ocv.get(SOC());
    }

    @Override
    double R() {
        return r.get(SOC());
    }

    /**
     * Applies Peukert's law, returns derating [0,1] for the given discharge
     * current.
     * 
     * The 20h capacity implies a 0.9A discharge rate.
     * 
     * https://en.wikipedia.org/wiki/Peukert's_law
     * 
     * Note this "derating" is only useful if the current draw is constant: it
     * models a time-dependent phenomenon in the battery (reactant migration), and
     * given time at lower discharge (or at rest), some of the "lost" capacity can
     * "reappear".
     */
    double peukert(double i) {
        if (i < 0.1)
            return 1.0;
        return Math.pow(i0 / i, k - 1);
    }

    void setC(double c) {
        if (Double.isNaN(c))
            throw new IllegalArgumentException();
        this.c = c;
    }

    /**
     * Open-circuit voltage as a function of charge state.
     * 
     * This comes from eyeballing
     * https://www.athensjournals.gr/technology/2016-3-3-4-Dost.pdf
     * transcribed here:
     * https://docs.google.com/spreadsheets/d/1D61uZiuB7fR6dDHDfRE-6lHsw0oD9cSROUWzGtkaWhY/edit?gid=850752207#gid=850752207
     */
    private static InterpolatingDoubleTreeMap makeOCV() {
        InterpolatingDoubleTreeMap ocv = new InterpolatingDoubleTreeMap();
        // This value makes sure that the battery doesn't go below zero.
        // In a real battery, there is still some tiny amount of voltage
        // that will "polarize" the battery after resting, but not produce
        // useful current. Nobody ever uses lead-acid batteries at such low SOC, it
        // damages them.
        ocv.put(0.00, 0.0);
        ocv.put(0.02, 10.70);
        ocv.put(0.05, 11.05);
        ocv.put(0.10, 11.25);
        ocv.put(0.20, 11.49);
        ocv.put(0.30, 11.67);
        ocv.put(0.40, 11.84);
        ocv.put(0.50, 12.00);
        ocv.put(0.60, 12.16);
        ocv.put(0.70, 12.32);
        ocv.put(0.80, 12.48);
        ocv.put(0.90, 12.64);
        ocv.put(1.00, 12.8);
        return ocv;
    }

    /**
     * Resistance as a function of charge state.
     * 
     * I used figures like these:
     * https://www.researchgate.net/figure/nternal-resistance-versus-SOC_fig2_312558387
     * https://www.biologic.net/documents/eis-high-frequencies-internal-resistance-battery-application-note-62/
     * transcribed here
     * https://docs.google.com/spreadsheets/d/1D61uZiuB7fR6dDHDfRE-6lHsw0oD9cSROUWzGtkaWhY/edit?gid=659397064#gid=659397064
     * I scaled the result to the label resistance of 16 mohm.
     */
    private static InterpolatingDoubleTreeMap makeR() {
        InterpolatingDoubleTreeMap r = new InterpolatingDoubleTreeMap();
        // This value prevents high power output at low SOC. I'm not sure
        // what the real value should be here. Nobody ever uses lead-acid
        // batteries at such low SOC, it damages them.
        r.put(0.0, 10.0);
        r.put(0.02, 0.07);
        r.put(0.05, 0.0532);
        r.put(0.1, 0.042);
        r.put(0.2, 0.0308);
        r.put(0.3, 0.0252);
        r.put(0.4, 0.0224);
        r.put(0.5, 0.021);
        r.put(0.6, 0.0196);
        r.put(0.7, 0.0182);
        r.put(0.8, 0.0175);
        r.put(0.9, 0.0168);
        r.put(1.0, 0.0168);
        return r;
    }

}
