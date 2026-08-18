package org.team100.lib.sensor.position.absolute;

import edu.wpi.first.math.MathUtil;

/** Contains no logic. */
public class MockRotaryPositionSensor implements RotaryPositionSensor {
    public double angle = 0;
    public double rate = 0;

    @Override
    public double getWrappedPositionRad() {
        return MathUtil.angleModulus(angle);
    }

    @Override
    public double getUnwrappedPositionRad() {
        return angle;
    }

    @Override
    public double getVelocityRad_S() {
        return rate;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void close() {
        //
    }
}
