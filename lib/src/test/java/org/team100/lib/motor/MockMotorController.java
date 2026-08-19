package org.team100.lib.motor;

import org.wpilib.hardware.motor.MotorController;

public class MockMotorController implements MotorController {
    public double speed;
    public boolean inverted;

    @Override
    public void setThrottle(double speed) {
        this.speed = speed;
    }

    @Override
    public double getThrottle() {
        return speed;
    }

    @Override
    public void setInverted(boolean isInverted) {
        inverted = isInverted;
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

    @Override
    public void disable() {
        //
    }

    // @Override
    // public void stopMotor() {
    //     speed = 0;
    // }
}
