package org.team100.lib.motor.ctre;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;

/**
 * Help TunerX see all the can buses.
 * 
 * https://github.com/wpilibsuite/SystemcoreTesting/issues/378
 */
public class CTREStartup {
    public static void start() {
        new CANBus(CANPort.CAN_S0);
        new CANBus(CANPort.CAN_S1);
        new CANBus(CANPort.CAN_S2);
        new CANBus(CANPort.CAN_S3);
        new CANBus(CANPort.CAN_S4);
    }

}
