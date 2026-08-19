package org.team100.lib.sensor.gyro;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.math.geometry.Rotation2d;

public class SystemcoreGyro implements Gyro {
    /**
     * Gyro data rate in Hz.
     * 
     * For the purpose of estimating sample noise, we assume the averaging period is
     * the same.
     */
    private static final double SAMPLE_RATE = 100;
    /**
     * Standard deviation of rate measurements in rad/s.
     */
    private static final double NOISE = 4e-4 * Math.sqrt(SAMPLE_RATE);
    /**
     * Standard deviation of bias in rad/s.
     */
    private static final double BIAS_NOISE = 1e-5;

    private final OnboardIMU m_imu;

    private final Rotation2dLogger m_log_yaw;
    private final DoubleLogger m_log_yaw_rate;
    private final Rotation2dLogger m_log_pitch;
    private final Rotation2dLogger m_log_roll;

    public SystemcoreGyro(LoggerFactory parent) {
        m_imu = new OnboardIMU(MountOrientation.FLAT);
        LoggerFactory log = parent.type(this);
        m_log_yaw = log.rotation2dLogger(Level.TRACE, "Yaw NWU (rad)");
        m_log_yaw_rate = log.doubleLogger(Level.TRACE, "Yaw Rate NWU (rad_s)");
        m_log_pitch = log.rotation2dLogger(Level.TRACE, "Pitch NWU (rad)");
        m_log_roll = log.rotation2dLogger(Level.TRACE, "Roll NWU (rad)");

    }

    @Override
    public double white_noise() {
        return NOISE;
    }

    @Override
    public double bias_noise() {
        return BIAS_NOISE;
    }

    @Override
    public Rotation2d getYawNWU() {
        Rotation2d r = m_imu.getRotation2d();
        m_log_yaw.log(() -> r);
        return r;
    }

    @Override
    public double getYawRateNWU() {
        double r = m_imu.getGyroRateZ();
        m_log_yaw_rate.log(() -> r);
        return r;
    }

    @Override
    public Rotation2d getPitchNWU() {
        Rotation2d r = new Rotation2d(m_imu.getAngleY());
        m_log_pitch.log(() -> r);
        return r;
    }

    @Override
    public Rotation2d getRollNWU() {
        Rotation2d r = new Rotation2d(m_imu.getAngleX());
        m_log_roll.log(() -> r);
        return r;
    }

    @Override
    public void periodic() {
        //
    }

}
