package org.team100.lib.sensor.position.absolute;

import java.util.function.Supplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.util.CanId;
import org.team100.lib.util.Math100;
import org.wpilib.math.util.MathUtil;

import com.reduxrobotics.sensors.canandmag.Canandmag;

public class ReduxPositionSensor implements RotaryPositionSensor {
    private static final boolean DEBUG = false;
    private static final double TWO_PI = 2.0 * Math.PI;

    private final Canandmag encoder;
    private final EncoderDrive m_drive;
    private final Supplier<Integer> m_turns;

    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_position_turns;
    private final DoubleLogger m_log_position_turns_offset;

    private double m_positionOffsetTurns;
    private int m_turnCount;
    private double m_prevWrappedPositionRad;

    public ReduxPositionSensor(
            LoggerFactory parent,
            CanId id,
            double inputOffsetTurns,
            EncoderDrive drive) {
        LoggerFactory log = parent.type(this);
        encoder = new Canandmag(id.id);
        m_positionOffsetTurns = Math100.throwIfOutOfRange(inputOffsetTurns, 0.0, 1.0);
        m_drive = drive;

        m_turns = Cache.of(this::wrap);
        m_log_position = log.doubleLogger(Level.COMP, "position (rad)");
        m_log_position_turns = log.doubleLogger(Level.COMP, "position (turns)");
        m_log_position_turns_offset = log.doubleLogger(Level.TRACE, "position (turns-offset)");
    }

    private int wrap() {
        double current = getWrappedPositionRad();
        double prev = m_prevWrappedPositionRad;
        if (DEBUG) {
            System.out.printf("wrap prev %6.3f curr %6.3f\n", prev, current);
        }
        m_prevWrappedPositionRad = current;
        double diff = current - prev;
        if (diff > Math.PI) {
            return --m_turnCount;
        }
        if (diff < -Math.PI) {
            return ++m_turnCount;
        }
        return m_turnCount;
    }

    private double getRad() {
        double posTurns = encoder.getAbsPosition();
        m_log_position_turns.log(() -> posTurns);

        double turnsMinusOffset = posTurns - m_positionOffsetTurns;
        m_log_position_turns_offset.log(() -> turnsMinusOffset);

        switch (m_drive) {
            case DIRECT:
                return MathUtil.angleModulus(turnsMinusOffset * TWO_PI);
            case INVERSE:
                return MathUtil.angleModulus(-1.0 * turnsMinusOffset * TWO_PI);
            default:
                throw new IllegalArgumentException();
        }
    }

    @Override
    public double getWrappedPositionRad() {
        double positionRad = getRad();
        m_log_position.log(() -> positionRad);
        return positionRad;
    }

    public int getTurns() {
        return m_turns.get();
    }

    @Override
    public double getUnwrappedPositionRad() {
        return getWrappedPositionRad() + TWO_PI * getTurns();
    }

    @Override
    public double getVelocityRad_S() {
        switch (m_drive) {
            case DIRECT:
                return 2 * Math.PI * encoder.getVelocity();
            case INVERSE:
                return -2 * Math.PI * encoder.getVelocity();
            default:
                throw new IllegalArgumentException();
        }
    }

    @Override
    public double getAccelerationRad_S2() {
        return 0;
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double x) {
        double posTurns = encoder.getAbsPosition();
        double newturns = x / TWO_PI;
        int newwraps = (int) (newturns - posTurns);
        double newpos = posTurns + newwraps;
        double newoffset = newpos - newturns;
        m_positionOffsetTurns = newoffset;
        m_turnCount = newwraps;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void close() {
        encoder.close();
    }

}