package org.team100.lib.motor;

import org.team100.lib.sensor.position.incremental.IncrementalEncoder;

public class AbstractMotor implements Motor {

    @Override
    public void play(double freq) {
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
    }

    @Override
    public void setDutyCycle(double output) {
    }

    @Override
    public void setVoltage(double voltage) {
    }

    @Override
    public void setCurrent(double current) {
    }

    @Override
    public void setVelocity(double velocityRad_S, double torqueNm) {
    }

    @Override
    public double getStatorCurrent() {
        return 0;
    }

    @Override
    public void setUnwrappedPosition(
            double positionRad, double velocityRad_S, double torqueNm) {

    }

    @Override
    public double R() {
        return 0;
    }

    @Override
    public double kT() {
        return 0;
    }

    @Override
    public double kE() {
        return 0;
    }

    @Override
    public IncrementalEncoder encoder() {
        return null;
    }

    @Override
    public void stop() {
    }

    @Override
    public void reset() {
    }

    @Override
    public void close() {
    }

    @Override
    public void periodic() {
    }

    @Override
    public double getSupplyCurrent() {
        // no current measurement
        return 0;
    }

};