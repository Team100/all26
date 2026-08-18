package org.team100.lib.sensor.position.absolute.wpi;

import java.util.function.Supplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;

/**
 * One of the kinds of absolute rotary position sensors directly connected to
 * the RoboRIO.
 */
public abstract class RoboRioRotaryPositionSensor implements RotaryPositionSensor {
    private static final boolean DEBUG = false;
    private static final double TWO_PI = 2.0 * Math.PI;

    private final double m_positionOffset;
    private final EncoderDrive m_drive;
    private final Supplier<Integer> m_turns;
    // LOGGERS
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_position_turns;
    private final DoubleLogger m_log_position_turns_offset;

    private int m_turnCount;
    private double m_prevWrappedPositionRad;

    protected RoboRioRotaryPositionSensor(
            LoggerFactory parent,
            double inputOffset,
            EncoderDrive drive) {
        LoggerFactory log = parent.type(this);
        m_positionOffset = Math100.throwIfOutOfRange(inputOffset, 0.0, 1.0);
        m_drive = drive;

        m_turns = Cache.of(this::wrap);
        m_log_position = log.doubleLogger(Level.COMP, "position (rad)");
        m_log_position_turns = log.doubleLogger(Level.COMP, "position (turns)");
        m_log_position_turns_offset = log.doubleLogger(Level.TRACE, "position (turns-offset)");
    }

    /**
     * Sensor ratio in the interval [0, 1].
     * Implementations should cache this.
     */
    protected abstract double getRatio();

    protected abstract double sensorMin();

    protected abstract double sensorMax();

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

    public int getTurns() {
        return m_turns.get();
    }

    public double getUnwrappedPositionRad() {
        return getWrappedPositionRad() + 2 * Math.PI * getTurns();
    }

    /** This should be nearly cached. */
    @Override
    public double getWrappedPositionRad() {
        double positionRad = getRad();
        m_log_position.log(() -> positionRad);
        return positionRad;
    }

    /**
     * map to full [0,1]
     */
    static double mapSensorRange(double ratio, double sensorMin, double sensorMax) {
        if (ratio < sensorMin) {
            ratio = sensorMin;
        }
        if (ratio > sensorMax) {
            ratio = sensorMax;
        }
        return (ratio - sensorMin) / (sensorMax - sensorMin);
    }

    /**
     * This should be nearly cached.
     * 
     * @return radians, [-pi, pi]
     */
    private double getRad() {
        double ratio = getRatio();

        double posTurns = mapSensorRange(ratio, sensorMin(), sensorMax());
        m_log_position_turns.log(() -> posTurns);

        double turnsMinusOffset = posTurns - m_positionOffset;
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



    /**
     * Always returns zero.
     * 
     * Extracting velocity from the absolute position sensor does not work.
     */
    @Override
    public double getVelocityRad_S() {
        return 0;
    }

}
