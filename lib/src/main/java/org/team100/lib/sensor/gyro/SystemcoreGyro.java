package org.team100.lib.sensor.gyro;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.DoubleCache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
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

    private final ObjectCache<Rotation2d> m_yaw;
    private final DoubleCache m_yawRate;
    private final ObjectCache<Rotation2d> m_pitch;
    private final ObjectCache<Rotation2d> m_roll;

    private final Rotation2dLogger m_log_yaw;
    private final DoubleLogger m_log_yaw_rate;
    private final Rotation2dLogger m_log_pitch;
    private final Rotation2dLogger m_log_roll;

    private final DoubleLogger m_log_accel_x;
    private final DoubleLogger m_log_accel_y;
    private final DoubleLogger m_log_accel_z;

    public SystemcoreGyro(LoggerFactory parent) {
        m_imu = new OnboardIMU(MountOrientation.FLAT);
        LoggerFactory log = parent.type(this);

        m_yaw = Cache.of(this::yaw);
        m_yawRate = Cache.ofDouble(this::yawRate);
        m_pitch = Cache.of(this::pitch);
        m_roll = Cache.of(this::roll);

        m_log_yaw = log.rotation2dLogger(Level.TRACE, "Yaw NWU (rad)");
        m_log_yaw_rate = log.doubleLogger(Level.TRACE, "Yaw Rate NWU (rad_s)");
        m_log_pitch = log.rotation2dLogger(Level.TRACE, "Pitch NWU (rad)");
        m_log_roll = log.rotation2dLogger(Level.TRACE, "Roll NWU (rad)");

        m_log_accel_x = log.doubleLogger(Level.TRACE, "Accel X (g)");
        m_log_accel_y = log.doubleLogger(Level.TRACE, "Accel Y (g)");
        m_log_accel_z = log.doubleLogger(Level.TRACE, "Accel Z (g)");

        LogPoller.register(this::log);
    }

    private void log() {
        m_log_yaw.log(() -> getYawNWU());
        m_log_yaw_rate.log(() -> getYawRateNWU());
        m_log_pitch.log(() -> getPitchNWU());
        m_log_roll.log(() -> getRollNWU());
        m_log_accel_x.log(m_imu::getAccelX);
        m_log_accel_y.log(m_imu::getAccelY);
        m_log_accel_z.log(m_imu::getAccelZ);
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
        return m_yaw.get();
    }

    private Rotation2d yaw() {
        return m_imu.getRotation2d();
    }

    @Override
    public double getYawRateNWU() {
        return m_yawRate.getAsDouble();
    }

    private double yawRate() {
        return m_imu.getGyroRateZ();
    }

    @Override
    public Rotation2d getPitchNWU() {
        return m_pitch.get();
    }

    private Rotation2d pitch() {
        return new Rotation2d(m_imu.getAngleY());
    }

    @Override
    public Rotation2d getRollNWU() {
        return m_roll.get();

    }

    private Rotation2d roll() {
        return new Rotation2d(m_imu.getAngleX());
    }
}
