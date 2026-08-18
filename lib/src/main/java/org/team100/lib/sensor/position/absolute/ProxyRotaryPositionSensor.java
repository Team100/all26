package org.team100.lib.sensor.position.absolute;

import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;

import edu.wpi.first.math.MathUtil;

/**
 * Proxies an IncrementalBareEncoder to produce a RotaryPositionSensor, by
 * taking the angle modulus.
 * 
 * Use it with the CombinedRotaryPositionSensor.
 */
public class ProxyRotaryPositionSensor implements RotaryPositionSensor {
    private final IncrementalBareEncoder m_encoder;
    private final double m_gearRatio;

    public ProxyRotaryPositionSensor(IncrementalBareEncoder encoder, double gearRatio) {
        m_encoder = encoder;
        m_gearRatio = gearRatio;
    }

    public ProxyRotaryPositionSensor(
            IncrementalBareEncoder encoder,
            double gearRatio,
            double initialPosition) {
        this(encoder, gearRatio);
        setEncoderPosition(initialPosition);
    }

    /**
     * Sets the incremental encoder position. This is only used to "zero" it, and
     * only done by the CombinedRotaryPositionSensor.
     * 
     * It is very slow: call it only on startup.
     */
    public void setEncoderPosition(double positionRad) {
        m_encoder.setUnwrappedEncoderPositionRad(positionRad * m_gearRatio);
    }

    @Override
    public double getWrappedPositionRad() {
        return MathUtil.angleModulus(getUnwrappedPositionRad());
    }

    @Override
    public double getUnwrappedPositionRad() {
        return m_encoder.getUnwrappedPositionRad() / m_gearRatio;
    }

    /**
     * Identical to RotaryMechanism.getVelocityRad_S()
     */
    @Override
    public double getVelocityRad_S() {
        return m_encoder.getVelocityRad_S() / m_gearRatio;
    }

    @Override
    public void periodic() {
        m_encoder.periodic();
    }

    @Override
    public void close() {
        m_encoder.close();
    }

}
