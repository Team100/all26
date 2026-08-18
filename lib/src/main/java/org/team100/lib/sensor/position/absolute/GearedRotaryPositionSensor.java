package org.team100.lib.sensor.position.absolute;

import edu.wpi.first.math.MathUtil;

/**
 * Represents an encoder which is not 1:1 with the mechanism.
 * 
 * IMPORTANT: this does not handle rollover, so it can't be used for control.
 * 
 * It may only be used for homing.
 * 
 * If you try to use this for control, the mechanism will behave in unexpected
 * and potentially dangerous ways, as the encoder crosses its zero point.
 * 
 * The intermediate gear goes too fast for rollover support to work -- there are
 * only ~5 ticks per intermediate shaft turn in good conditions, so in "overrun"
 * conditions we could see only a few ticks, and we could easily miss edges.
 */
public class GearedRotaryPositionSensor implements RotaryPositionSensor {
    private final RotaryPositionSensor m_delegate;
    private final double m_ratio;

    public GearedRotaryPositionSensor(
            RotaryPositionSensor delegate,
            double ratio) {
        m_delegate = delegate;
        m_ratio = ratio;
    }

    @Override
    public double getWrappedPositionRad() {
        return MathUtil.angleModulus(getUnwrappedPositionRad());
    }

    @Override
    public double getUnwrappedPositionRad() {
        return m_delegate.getWrappedPositionRad() / m_ratio;
    }

    @Override
    public double getVelocityRad_S() {
        return m_delegate.getVelocityRad_S() / m_ratio;
    }

    @Override
    public void periodic() {
        m_delegate.periodic();
    }

    @Override
    public void close() {
        m_delegate.close();
    }

}
