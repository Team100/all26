package org.team100.lib.servo;

public class MockLinearVelocityServo implements LinearVelocityServo {
    double m_setpoint;

    @Override
    public void reset() {
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
    }

    @Override
    public void setVelocityDirect(double setpoint, double setpoint_2) {
        m_setpoint = setpoint;
    }

    @Override
    public double getVelocity() {
        return m_setpoint;
    }

    @Override
    public boolean atGoal() {
        return true;
    }

    @Override
    public void stop() {
    }

    @Override
    public double getDistance() {
        throw new UnsupportedOperationException("Unimplemented method 'getDistance'");
    }

    @Override
    public void periodic() {
    }

    @Override
    public void play(double freq) {
    }

    @Override
    public void setVelocityProfiled(double setpointM_S) {
        m_setpoint = setpointM_S;
    }

    @Override
    public boolean atSetpoint() {
        return false;
    }

    @Override
    public boolean profileDone() {
        return false;
    }

    @Override
    public void close() {
    }
}
