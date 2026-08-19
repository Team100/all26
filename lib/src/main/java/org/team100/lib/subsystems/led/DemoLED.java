package org.team100.lib.subsystems.led;

import org.team100.lib.subsystems.lynxmotion_arm.AxisCalibrator;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.util.Color;

/**
 * Drives the LED strip attached to the Lynxmotion arm trainer, useful for
 * initial bring-up of the RoboRIO.
 * 
 * https://www.adafruit.com/product/3811
 * 
 */
public class DemoLED extends SubsystemBase {
    private static final int LENGTH = 30;
    private final int STEPS = 2;
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    private int m_position = 0;
    private int m_inc = 1;
    private int m_counter = 0;

    public DemoLED() {
        m_led = new AddressableLED(9);
        m_buffer = new AddressableLEDBuffer(LENGTH);
        m_led.setLength(LENGTH);
        // m_led.start();
    }

    public Command sweep() {
        return run(this::sweepStep);
    }

    public Command indicateCalibration(AxisCalibrator calibrator) {
        return run(() -> set((int) (calibrator.getPosition() * (LENGTH-1))));
    }

    private void set(int x) {
        for (int i = 0; i < LENGTH; ++i) {
            if (i == x) {
                m_buffer.setLED(i, Color.ORANGE_RED);
            } else {
                m_buffer.setLED(i, Color.BLACK);
            }
        }
        m_led.setData(m_buffer);
    }

    private void sweepStep() {
        m_counter++;
        if (m_counter < STEPS) {
            return;
        }
        m_counter = 0;

        if (m_position == LENGTH - 1) {
            // we're at the end, so go back
            m_inc = -1;
        } else if (m_position == 0) {
            // we're at the start, so go forward
            m_inc = 1;
        }
        m_position += m_inc;
        set(m_position);
    }

}
