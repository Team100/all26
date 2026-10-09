package org.team100.frc2026.robot;

import org.team100.lib.coherence.Takt;

public class Prewarmer {
    public static void init(Machinery machinery) {
        System.out.println("\n*** PREWARM START");
        double startS = Takt.actual();
        //
        // actual warming goes here
        //
        double endS = Takt.actual();
        // don't count this delay in the ET
        waitForFPGA();
        System.out.printf("\n*** PREWARM END ET: %f\n", endS - startS);
    }

    /**
     * The duty cycle encoder produces garbage for a few seconds so sleep.
     */
    public static void waitForFPGA() {
        try {
            System.out.println("Waiting for DutyCycle sensors to work ...");
            Thread.sleep(1000);
            System.out.println("Waiting for DutyCycle sensors to work ...");
            Thread.sleep(1000);
            System.out.println("Waiting for DutyCycle sensors to work ...");
            Thread.sleep(1000);
            System.out.println("Done!");
        } catch (InterruptedException e) {
        }
    }

}
