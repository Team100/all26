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
        System.out.printf("\n*** PREWARM END ET: %f\n", endS - startS);
    }

}
