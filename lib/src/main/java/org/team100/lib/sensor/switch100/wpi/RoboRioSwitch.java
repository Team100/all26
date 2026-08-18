package org.team100.lib.sensor.switch100.wpi;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.wpilibj.DigitalInput;

/** Represents the state of a RoboRIO digital input. */
public class RoboRioSwitch {
    private final BooleanLogger m_log;
    private final DigitalInput m_sensor;

    public RoboRioSwitch(LoggerFactory log, RoboRioChannel channel) {
        m_log = log.type(this).booleanLogger(Level.TRACE, "state");
        m_sensor = new DigitalInput(channel.channel);
    }

    public boolean get() {
        boolean state = m_sensor.get();
        m_log.log(() -> state);
        return state;
    }
}
