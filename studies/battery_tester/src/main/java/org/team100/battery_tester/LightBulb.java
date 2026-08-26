package org.team100.battery_tester;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * This is a basic physics model of a light bulb, used for feedforward for the
 * battery tester.
 * 
 * The interesting thing about a light bulb is how strongly the resistance
 * depends on the temperature. It's at least a 10x effect. This leads to a
 * transient "inrush current" when the bulb is turned on; we should be careful
 * to turn the bulbs on gradually, to avoid excess current in this transient.
 * 
 * https://www.electronicsfaq.com/2013/11/measuring-hot-resistance-and-cold.html
 * 
 * The emissivity of the filament also varies about 10x over the operating
 * temperature range.
 * 
 * https://amu.hal.science/hal-01690700/document
 * 
 * TODO: calibrate this model.
 * 
 * To keep things simple, this is a steady-state model.
 * 
 * TODO: add transient behavior.
 * 
 * For now, this models just one thing, the collection of light bulbs
 * used in the battery tester. The specific bulb used is here:
 * 
 * https://www.lightbulbs.com/product/sunlite-03310
 * 
 * Its design point is 3200K.
 * 
 * TODO: model more kinds of light bulbs.
 * 
 * At full output, each bulb draws 12.5 A at 12.0 V,
 * for 150 W, so the resistance of each bulb is about 0.96 ohm.
 * 
 * Total current: 12.5 amp * 15 = 187 amp.
 * Total resistance: E/I = 12 / 187 = 0.064171 ohm.
 * 
 * At the design point of 3200K, the tungsten resistivity is 99.54 microohm-cm,
 * i.e. 9.954e-7 ohm-m.
 * The filament area divided by length (A/l) is 9.954e-7 ohm-m/6.4171e-2 ohm
 * or about 1.5512e-5 m.
 * 
 * https://en.wikipedia.org/wiki/Electrical_resistivity_and_conductivity#Definition
 * 
 * Assume all the light bulb output is radiated, following the Stefan-Boltzmann
 * law, A*e*sigma*T^4.
 * 
 * The design output of the bulb is 150W at 3200K, where the emissivity is
 * 0.344, so 150 = Asigma * 0.344 * 3200^4. So Asigma is 4.15846e-12 W/K^4.
 * 
 * There are 15 bulbs, so the full value of Asigma for the whole array is
 * 6.23769e-11 W/K^4
 * 
 * TODO: add conduction.
 * 
 * https://en.wikipedia.org/wiki/Stefan%E2%80%93Boltzmann_law
 */
public class LightBulb {
    /** Operating point of the bulbs. */
    public record Op(double v, double i) {
    }

    private static final boolean DEBUG = false;
    /**
     * Filament cross-section divided by length, in meters, for the whole array.
     * To get resistance, divide resistivity (in ohm-m) by this number.
     */
    private static final double AoverL = 1.5512e-5;

    /**
     * Radiation constant, W/K^4, for the whole array.
     */
    private static final double Asigma = 6.23770e-11;

    /**
     * Emissivity as a function of temperature.
     * 
     * key: temperature, kelvin
     * value: total emissivity, [0, 1]
     */
    private final InterpolatingDoubleTreeMap epsilon;

    /**
     * Resistivity as a function of temperature.
     * 
     * key: temperature, kelvin
     * value: resistivity, microohm-cm
     */
    private final InterpolatingDoubleTreeMap rho;

    public LightBulb() {
        epsilon = makeE();
        rho = makeR();
    }

    /** Operating point of the bulbs for the given power (watts). */
    public Op operatingPoint(double p) {
        // Temperature required to radiate the required powper
        double t = temperature(p);
        // Resistance at that temperature.
        double r = R(t);
        // Ohm and Joule's laws, P = V^2 / R
        double v = Math.sqrt(p * r);
        // Joule's law, P = I^2 R
        double i = Math.sqrt(p / r);
        return new Op(v, i);
    }

    /**
     * Current drawn at voltage.
     * 
     * This is just Ohm's law, but it involves iteration, because the resistance is
     * strongly dependent on temperature, which is strongly dependent on current.
     */
    public double IforV(double v) {
        if (Double.isNaN(v))
            throw new IllegalArgumentException();
        double t = 0;
        for (int j = 0; j < 100; ++j) {
            double r = R(t);
            double p = v * v / r;
            double t0 = t;
            t = temperature(p);
            if (Math.abs(t - t0) < 1) {
                double i = v / r;
                if (DEBUG)
                    System.out.printf("LightBulb.IforV: i %f p %f\n", i, p);
                // System.out.printf("LightBulb.IforV %d\n", j);
                return i;
            }
        }
        System.out.printf(
                "LightBulb: current failed to converge for voltage %f\n", v);
        return v / R(t);
    }

    /**
     * Temperature (kelvin) to radiate power (watts), across the full array (15
     * bulbs).
     * 
     * This is Stefan-Boltzman, solving for temperature. Note because
     * emissivity depends on temperature, this involves (a tiny bit of) iteration.
     */
    public double temperature(double p) {
        if (Double.isNaN(p))
            throw new IllegalArgumentException();
        double t = 0;
        for (int i = 0; i < 100; ++i) {
            double t0 = t;
            t = Math.pow(p / (Asigma * epsilon.get(t0)), 0.25);
            if (Math.abs(t - t0) < 1)
                return t;
        }
        System.out.printf("Lightbulb: temperature failed to converge for power %f\n", p);
        return t;
    }

    /**
     * Resistance (ohms) at temperature (kelvin), of the full bulb array (15 bulbs).
     */
    double R(double t) {
        double rhomOhmCm = rho.get(t);
        double rhoOhmM = rhomOhmCm * 1e-8;
        return rhoOhmM / AoverL;
    }

    /**
     * Radiation (watts) at temperature (kelvin), of the full bulb array (15 bulbs).
     * 
     * This is exactly the Stefan-Boltzmann equation:
     * 
     * P = A * sigma * e * T^4.
     */
    double radiation(double t) {
        double e = epsilon.get(t);
        return Asigma * e * Math.pow(t, 4);
    }

    /**
     * Data:
     * https://www.semanticscholar.org/paper/Resistance-and-Radiation-of-Tungsten-as-a-Function-Forsythe-Watson/84fdeb704a4781a6dcf84586502f0d287449e814/figure/0
     * 
     * Transcribed, with a tiny bit of interpolation, here:
     * https://docs.google.com/spreadsheets/d/1D61uZiuB7fR6dDHDfRE-6lHsw0oD9cSROUWzGtkaWhY
     */
    private static InterpolatingDoubleTreeMap makeE() {
        InterpolatingDoubleTreeMap e = new InterpolatingDoubleTreeMap();
        e.put(273.0, 0.022);
        e.put(293.0, 0.023);
        e.put(300.0, 0.024);
        e.put(400.0, 0.034);
        e.put(500.0, 0.042);
        e.put(600.0, 0.052);
        e.put(700.0, 0.062);
        e.put(800.0, 0.074);
        e.put(900.0, 0.089);
        e.put(1000.0, 0.105);
        e.put(1100.0, 0.121);
        e.put(1200.0, 0.138);
        e.put(1300.0, 0.156);
        e.put(1400.0, 0.174);
        e.put(1500.0, 0.192);
        e.put(1600.0, 0.207);
        e.put(1700.0, 0.222);
        e.put(1800.0, 0.236);
        e.put(1900.0, 0.248);
        e.put(2000.0, 0.259);
        e.put(2100.0, 0.269);
        e.put(2200.0, 0.278);
        e.put(2300.0, 0.286);
        e.put(2400.0, 0.294);
        e.put(2500.0, 0.301);
        e.put(2600.0, 0.309);
        e.put(2700.0, 0.315);
        e.put(2800.0, 0.321);
        e.put(2900.0, 0.329);
        e.put(3000.0, 0.334);
        e.put(3100.0, 0.339);
        e.put(3200.0, 0.344);
        e.put(3300.0, 0.349);
        e.put(3400.0, 0.354);
        e.put(3500.0, 0.359);
        e.put(3600.0, 0.364);
        return e;
    }

    /**
     * Data is from here:
     * https://hypertextbook.com/facts/2004/DeannaStewart.shtml
     * 
     * Transcribed here:
     * https://docs.google.com/spreadsheets/d/1D61uZiuB7fR6dDHDfRE-6lHsw0oD9cSROUWzGtkaWhY
     * 
     * Corroboration (similar, not precisely the same)
     * https://www.semanticscholar.org/paper/Resistance-and-Radiation-of-Tungsten-as-a-Function-Forsythe-Watson/84fdeb704a4781a6dcf84586502f0d287449e814/figure/0
     */
    private static InterpolatingDoubleTreeMap makeR() {
        InterpolatingDoubleTreeMap R = new InterpolatingDoubleTreeMap();
        R.put(300.0, 5.65); // around room temperature
        R.put(400.0, 8.06);
        R.put(500.0, 10.56);
        R.put(600.0, 13.23);
        R.put(700.0, 16.09);
        R.put(800.0, 19.00);
        R.put(900.0, 21.94);
        R.put(1000.0, 24.93);
        R.put(1100.0, 27.94);
        R.put(1200.0, 30.98);
        R.put(1300.0, 34.08);
        R.put(1400.0, 37.19);
        R.put(1500.0, 40.36);
        R.put(1600.0, 43.55);
        R.put(1700.0, 46.78);
        R.put(1800.0, 50.05);
        R.put(1900.0, 53.35);
        R.put(2000.0, 56.67);
        R.put(2100.0, 60.06);
        R.put(2200.0, 63.48);
        R.put(2300.0, 66.91);
        R.put(2400.0, 70.39);
        R.put(2500.0, 73.91);
        R.put(2600.0, 77.49);
        R.put(2700.0, 81.04);
        R.put(2800.0, 84.70);
        R.put(2900.0, 88.33);
        R.put(3000.0, 92.04);
        R.put(3100.0, 95.76);
        R.put(3200.0, 99.54); // design point at 12.0 volts (which we never achieve)
        R.put(3300.0, 103.30);
        R.put(3400.0, 107.20);
        R.put(3500.0, 111.10);
        R.put(3600.0, 115.00);
        return R;
    }
}
