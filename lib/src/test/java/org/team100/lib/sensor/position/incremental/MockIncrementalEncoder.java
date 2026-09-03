package org.team100.lib.sensor.position.incremental;

/** Contains no logic. */
public class MockIncrementalEncoder implements IncrementalEncoder {
    public double position = 0;
    public double velocity = 0;
    public double acceleration = 0;

    @Override
    public double getUnwrappedPositionRad() {
        return position;
    }

    @Override
    public double getVelocityRad_S() {
        return velocity;
    }

    @Override
    public double getAccelerationRad_S2() {
        return acceleration;
    }

    @Override
    public void close() {
        //
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double motorPositionRad) {
        position = motorPositionRad;
    }

    @Override
    public void periodic() {
        //
    }

}
