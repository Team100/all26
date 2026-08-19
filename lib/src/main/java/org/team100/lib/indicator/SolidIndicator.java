package org.team100.lib.indicator;

import static org.wpilib.command2.Commands.repeatingSequence;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team100.lib.util.RoboRioChannel;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.util.Color;

/**
 * An LED indicator that is all the same color.
 * 
 * There are two kinds of indicator:
 * 
 * * steady indicates a continuing state, e.g. "game piece in grip."
 * 
 * * blinking indicates an event, e.g. "game piece just acquired."
 * 
 * The steady states are supplied to state(), which uses the default command
 * internally.
 * 
 * The events are registered with event(), which uses Triggers.
 * 
 * Also serves as an example of using the CommandScheduler.
 * 
 * Some guidance for indicator design:
 * 
 * * Show things that the drive team otherwise would have a hard time knowing,
 * e.g. "item acquired," or "localization confidence is high."
 * 
 * * Don't duplicate things that the drive team already knows, e.g. "trying to
 * intake" is just echoing a button.
 * 
 * Resources regarding signal design:
 * 
 * https://www.denix.osd.mil/soh/denix-files/sites/21/2016/03/02_MIL-STD-1472F-Human-Engineering.pdf
 * https://www.faa.gov/regulations_policies/rulemaking/committees/documents/media/TAEas-wcal-04232002.pdf
 */
public class SolidIndicator extends SubsystemBase {
    private static final double BLINK_DURATION = 0.06;
    private static final int EVENT_DURATION = 1;
    private final AddressableLED m_led;
    private final int m_length;
    private final Map<Color, AddressableLEDBuffer> m_buffers;

    public SolidIndicator(RoboRioChannel channel, int length) {
        m_led = new AddressableLED(channel.channel);
        m_length = length;
        m_buffers = new HashMap<>();
        m_led.setLength(length);
        // m_led.start();
    }

    /**
     * Show the supplied color.
     */
    public void state(Supplier<Color> color) {
        setDefaultCommand(
                run(() -> set(color.get()))
                        .ignoringDisable(true)
                        .withName("Indicator default"));
    }

    /**
     * Trigger on true, blink the color a few times.
     */
    public void event(BooleanSupplier condition, Color color) {
        new Trigger(condition).onTrue(
                blink(color)
                        .ignoringDisable(true)
                        .withTimeout(EVENT_DURATION)
                        .withName("Indicator event"));
    }

    public void close() {
        m_led.close();
    }

    /////////////////////////////////////////////////////////////

    private Command blink(Color color) {
        return repeatingSequence(
                once(color),
                once(Color.BLACK));
    }

    private Command once(Color color) {
        return run(() -> set(color)).withTimeout(BLINK_DURATION);
    }

    private void set(Color color) {
        m_led.setData(forColor(color));
    }

    private AddressableLEDBuffer forColor(Color color) {
        return m_buffers.computeIfAbsent(color, this::fill);
    }

    private AddressableLEDBuffer fill(Color color) {
        AddressableLEDBuffer buf = new AddressableLEDBuffer(m_length);
        for (int i = 0; i < m_length; ++i) {
            buf.setLED(i, color);
        }
        return buf;
    }

}
