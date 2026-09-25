package org.team100.lib.subsystems.lynxmotion_arm;

import static org.team100.lib.util.TriggerUtil.onTrue;

import org.team100.lib.subsystems.led.DemoLED;
import org.wpilib.driverstation.Gamepad;

/** Sets up the axis calibrator to use the Lynxmotion arm. */
public class CalibratorSetup implements Runnable {
    private final AxisCalibrator m_calibrator;

    public CalibratorSetup(Gamepad m_controller, DemoLED m_led) {
        m_calibrator = new AxisCalibrator(5);
        onTrue(m_controller::getWestFaceButton,
                m_calibrator.step());
        m_led.setDefaultCommand(m_led.indicateCalibration(m_calibrator));
    }

    @Override
    public void run() {
        //
    }

}
