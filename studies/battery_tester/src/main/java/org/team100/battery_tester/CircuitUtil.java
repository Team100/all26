package org.team100.battery_tester;

/** Simulates bulb and battery together. */
public class CircuitUtil {
    /** Operating point of the controllers. */
    public record Op(double inputV, double inputI, double outputV, double outputI) {
    }

    private static final boolean DEBUG = false;
    private final LightBulb lightbulb;
    private final Battery battery;

    public CircuitUtil(LightBulb l, Battery b) {
        lightbulb = l;
        battery = b;
    }

    /**
     * Return operating point for the given duty cycle.
     * 
     * Iterates due to the strong dependence of parameters on the operating point.
     */
    public Op operatingPointForDutyCycle(double d) {
        // start with open-circuit voltage
        double inputV = battery.V0();
        double outputV = d * inputV;
        double outputI = lightbulb.IforV(outputV);
        double outputP = outputV * outputI;
        double inputI = outputP / inputV;

        if (inputV < 1e-3) {
            // can't sag below zero
            return new Op(inputV, inputI, outputV, outputI);
        }

        // System.out.printf("d %f\n", d);
        for (int j = 0; j < 100; ++j) {
            // System.out.printf("j %d\n", j);
            outputV = d * inputV;
            // System.out.printf("outputV %f\n", outputV);
            outputI = lightbulb.IforV(outputV);
            outputP = outputV * outputI;
            // Battery current required at the previous battery voltage guess.
            inputI = outputP / inputV;
            // double v0 = inputV;
            // New voltage estimate.
            inputV = battery.V(inputI);
            // System.out.printf("inputV %f\n", inputV);
            // Avoid oscillation, at the cost of more iteration.
            // inputV = 0.1 * inputV + 0.9 * v0;
            double inputP = inputV * inputI;
            if (Math.abs(inputP - outputP) < 0.001) {
                if (DEBUG)
                    System.out.printf(
                            "CircuitUtil.batteryVoltageForDutycycle: success inputV %f inputI %f outputV %f outputI %f p %f\n",
                            inputV, inputI, outputV, outputI, outputP);
                // System.out.printf("solved %d\n", j);
                return new Op(inputV, inputI, outputV, outputI);
            }
        }

        System.out.printf(
                "CircuitUtil.batteryVoltageForDutycycle: fail soc %f, duty cycle %f\n", battery.SOC(), d);
        return new Op(inputV, inputI, outputV, outputI);
    }
}
