package org.team100.lib.motor.ctre;

import com.ctre.phoenix6.CANBus;

/**
 * Help TunerX see all the can buses.
 * 
 * https://github.com/wpilibsuite/SystemcoreTesting/issues/378
 */
public class CTREStartup {
    public static void start() {
        CANBus.systemcore(0);
        CANBus.systemcore(1);
        CANBus.systemcore(2);
        CANBus.systemcore(3);
        CANBus.systemcore(4);
    }

}
