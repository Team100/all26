package org.team100.battery_tester;

/**
 * Simple model of a battery, a fixed voltage source and a resistor.
 * 
 * This model is intended for feedforward; it doesn't need to be correct.
 */
public class EternalBattery extends Battery {
    @Override
    double V0() {
        return 12.6;
    }

    @Override
    double R() {
        return 0.02;
    }

    @Override
    double SOC() {
        return 1.0;
    }
}
