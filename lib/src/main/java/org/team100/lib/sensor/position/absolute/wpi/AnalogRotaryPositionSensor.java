package org.team100.lib.sensor.position.absolute.wpi;

import java.util.function.DoubleSupplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Absolute rotary position sensor using ratiometric analog input.
 * 
 * This type of sensor should not be used: the analog input is sensitive to
 * noise. It's only used in the legacy "arm" code (because there's no reason to
 * change it), and in the swerve factory, (in order to preserve the
 * selectable-encoder code paths).
 * 
 * Relies on Cache and Takt, so you must put Cache.refresh() and Takt.update() in
 * Robot.robotPeriodic().
 */
public class AnalogRotaryPositionSensor extends RoboRioRotaryPositionSensor {
    private final AnalogInput m_input;
    private final DoubleSupplier m_voltage;
    private final DoubleSupplier m_rail;
    private final DoubleLogger m_log_voltage;
    private final DoubleLogger m_log_ratio;

    public AnalogRotaryPositionSensor(
            LoggerFactory parent,
            RoboRioChannel channel,
            double inputOffset,
            EncoderDrive drive) {
        super(parent, inputOffset, drive);
        LoggerFactory log = parent.type(this);
        m_input = new AnalogInput(channel.channel);
        m_voltage = Cache.ofDouble(m_input::getVoltage);
        m_rail = Cache.ofDouble(RobotController::getVoltage5V);
        m_log_voltage = log.doubleLogger(Level.TRACE, "voltage");
        m_log_ratio = log.doubleLogger(Level.TRACE, "ratio");
        log.intLogger(Level.COMP, "channel").log(m_input::getChannel);
    }

    @Override
    protected double sensorMin() {
        return 0.0;
    }

    @Override
    protected double sensorMax() {
        return 1.0;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void close() {
        m_input.close();
    }

    /** Cached, almost. */
    @Override
    protected double getRatio() {
        double voltage = m_voltage.getAsDouble();
        double ratio = voltage / m_rail.getAsDouble();
        m_log_voltage.log(() -> voltage);
        m_log_ratio.log(() -> ratio);
        return ratio;
    }
}
