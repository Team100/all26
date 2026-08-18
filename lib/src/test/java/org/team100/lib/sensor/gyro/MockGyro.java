package org.team100.lib.sensor.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public class MockGyro implements Gyro {
    public Rotation2d rotation = Rotation2d.kZero;
    public double rate = 0;

    @Override
    public double white_noise() {
        return 4e-4;
    }

    @Override
    public double bias_noise() {
        return 1e-5;
    }

    @Override
    public Rotation2d getYawNWU() {
        return rotation;
    }

    @Override
    public double getYawRateNWU() {
        return rate;
    }

    @Override
    public Rotation2d getPitchNWU() {
        return Rotation2d.kZero;
    }

    @Override
    public Rotation2d getRollNWU() {
        return Rotation2d.kZero;
    }

    @Override
    public void periodic() {
        //
    }
}