package org.team100.lib.servo;

import org.team100.lib.state.StateR1;

public class MockAngularPositionServo implements AngularPositionServo {
    double m_setpoint;

    @Override
    public void reset() {
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
    }

    @Override
    public void setPositionProfiled(double goalRad) {
    }

    @Override
    public void setPositionDirect(double goalRad, double velocityRad_S) {
        m_setpoint = goalRad;
    }

    @Override
    public double getWrappedPositionRad() {
        return m_setpoint;
    }

    @Override
    public double getUnwrappedPositionRad() {
        return 0;
    }

    @Override
    public StateR1 getUnwrappedGoal() {
        return null;
    }

    @Override
    public boolean validSetpoint() {
        return true;
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
    public boolean atGoal() {
        return false;
    }

    @Override
    public void stop() {
    }

    @Override
    public void close() {
    }

    @Override
    public void periodic() {
    }

    @Override
    public void play(double freq) {
    }

    @Override
    public void actuateWithProfile(double unwrappedGoalX) {
    }

    @Override
    public void actuateDirect(double unwrappedSetpoint) {
    }

    @Override
    public void setVelocity(double v) {
    }

}
