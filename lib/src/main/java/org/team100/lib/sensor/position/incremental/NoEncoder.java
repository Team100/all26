package org.team100.lib.sensor.position.incremental;

/**
 * An encoder implementation that doesn't do anything.
 * This is useful if you want to make a mechanism (so you can have gears) but
 * you are using a primitive motor controller and no separate encoder anywhere.
 */
public class NoEncoder implements IncrementalEncoder {

    @Override
    public double getUnwrappedPositionRad() {
        return 0;
    }

    @Override
    public double getVelocityRad_S() {
        return 0;
    }

    @Override
    public double getAccelerationRad_S2() {
        return 0;
    }

    @Override
    public void close() {
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double motorPositionRad) {
    }

    @Override
    public void periodic() {
    }

}
