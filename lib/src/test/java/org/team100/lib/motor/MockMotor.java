package org.team100.lib.motor;

import org.team100.lib.config.Friction;
import org.team100.lib.sensor.position.incremental.IncrementalEncoder;

public class MockMotor implements Motor, IncrementalEncoder {
    public double output = 0;
    /** rad */
    public double position = 0;
    /** rad/s */
    public double velocity = 0;
    /** rad/s^2 */
    public double acceleration = 0;
    /** Nm */
    public double torque = 0;

    /** These is for testing feedforwards. */
    public double ffVolts;
    public double frictionFFVolts;
    public double backEMFVolts;
    public double torqueFFVolts;
    private final Friction m_friction;

    public MockMotor(Friction friction) {
        m_friction = friction;
    }

    @Override
    public void setDutyCycle(double output) {
        this.output = output;
    }

    @Override
    public void setVoltage(double volts) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setCurrent(double current) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setVelocity(double motorRad_S, double torqueNm) {
        velocity = motorRad_S;
        torque = torqueNm;
    }

    @Override
    public void setUnwrappedPosition(
            double motorRad, double motorRad_S, double torqueNm) {
        position = motorRad;
        velocity = motorRad_S;
        torque = torqueNm;

        frictionFFVolts = m_friction.frictionFFVolts(motorRad_S);
        backEMFVolts = backEMFVoltage(motorRad_S);
        torqueFFVolts = getTorqueFFVolts(torqueNm);
        ffVolts = backEMFVolts + frictionFFVolts + torqueFFVolts;
    }

    @Override
    public double R() {
        return 0.1;
    }

    @Override
    public double kT() {
        return 0.02;
    }

    @Override
    public double kE() {
        // 60 * 12 / (6000 * 2 * pi) volt-sec/rad
        return 0.0190996;
    }

    @Override
    public IncrementalEncoder encoder() {
        return this;
    }

    @Override
    public void stop() {
        this.output = 0;
        this.velocity = 0;
    }

    @Override
    public void reset() {
        //
    }

    @Override
    public void close() {
        //
    }

    @Override
    public double getUnwrappedPositionRad() {
        return this.position;
    }

    @Override
    public double getVelocityRad_S() {
        return this.velocity;
    }

    @Override
    public double getAccelerationRad_S2() {
        return this.acceleration;
    }

    @Override
    public double getStatorCurrent() {
        return 0;
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        this.position = positionRad;
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
    }

    @Override
    public void periodic() {
    }

    @Override
    public void play(double freq) {
    }

    @Override
    public double getSupplyCurrent() {
        // no current measurement
        return 0;
    }

}
