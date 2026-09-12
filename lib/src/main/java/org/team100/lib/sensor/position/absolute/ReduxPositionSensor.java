package org.team100.lib.sensor.position.absolute;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.util.CanId;
import org.wpilib.math.util.MathUtil;

import com.reduxrobotics.sensors.canandmag.Canandmag;

public class ReduxPositionSensor implements RotaryPositionSensor {
    private final Canandmag encoder;
    private final double m_positionOffset;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_position_turns;
    private final DoubleLogger m_log_position_turns_offset;

    public ReduxPositionSensor(
            LoggerFactory parent, CanId id, double inputOffset) {
        LoggerFactory log = parent.type(this);
        encoder = new Canandmag(id.id);
        m_positionOffset = inputOffset;
        m_log_position = log.doubleLogger(Level.COMP, "position (rad)");
        m_log_position_turns = log.doubleLogger(Level.COMP, "position (turns)");
        m_log_position_turns_offset = log.doubleLogger(Level.TRACE, "position (turns-offset)");
    }

    @Override
    public double getWrappedPositionRad() {
        double positionRad = getUnwrappedPositionRad();
        m_log_position.log(() -> positionRad);
        return MathUtil.angleModulus(positionRad);
    }

    @Override
    public double getUnwrappedPositionRad() {
        double position = encoder.getPosition();
        m_log_position_turns.log(() -> position);
        double d = position - m_positionOffset;
        m_log_position_turns_offset.log(() -> d);
        return -2 * Math.PI * d;
    }

    @Override
    public double getVelocityRad_S() {
        return -2 * Math.PI * encoder.getVelocity();
    }

    @Override
    public double getAccelerationRad_S2() {
        return 0;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void close() {
        encoder.close();
    }

}
