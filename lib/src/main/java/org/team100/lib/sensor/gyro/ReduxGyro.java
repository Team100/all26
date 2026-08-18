package org.team100.lib.sensor.gyro;

import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;
import org.team100.lib.util.CanId;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroFaults;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import com.reduxrobotics.sensors.canandgyro.QuaternionFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Redux gyro folk say that the measurements provided are within 1ms of the
 * CAN packet being sent, and the CAN latency is quite low, but the CAN packet
 * might be received sometime before the RoboRIO interrupt fires; the logic
 * here corrects for that difference.
 */
public class ReduxGyro implements Gyro {
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

    private final Canandgyro m_gyro;

    // LOGGERS
    private final DoubleLogger m_log_age;
    private final Rotation2dLogger m_log_yaw;
    private final DoubleLogger m_log_yaw_rate;
    private final Rotation2dLogger m_log_pitch;
    private final Rotation2dLogger m_log_roll;

    public ReduxGyro(LoggerFactory parent, CanId canID) {
        LoggerFactory log = parent.type(this);
        m_gyro = new Canandgyro(canID.id);
        m_gyro.clearStickyFaults();

        // Both position and velocity should be reasonably fresh.
        CanandgyroSettings settings = new CanandgyroSettings();
        settings.setAngularPositionFramePeriod(1 / SAMPLE_RATE);
        settings.setAngularVelocityFramePeriod(1 / SAMPLE_RATE);

        if (!m_gyro.setSettings(settings, 0.1)) {
            System.out.println("WARNING: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            System.out.println("WARNING: !!                                          !!");
            System.out.println("WARNING: !!           GYRO SETTING FAILED!           !!");
            System.out.println("WARNING: !!                                          !!");
            System.out.println("WARNING: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }
        m_gyro.clearStickyFaults();
        m_gyro.setYaw(0);

        m_log_age = log.doubleLogger(Level.TRACE, "position frame age (s)");
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

    /** This is latency-compensated to the current Takt time. */
    @Override
    public Rotation2d getYawNWU() {
        final QuaternionFrame q = m_gyro.getAngularPositionFrame();
        final double t = q.getTimestamp();
        final double yaw = q.getYaw();
        final double rate = m_gyro.getAngularVelocityYaw();
        double now = Takt.get();
        double dt = now - t;
        m_log_age.log(() -> now - t);
        // It's ok if takt is slightly behind the gyro, in case a CAN packet came in
        // before we got here.
        if (dt < -0.04) {
            dt = 0;
        }
        // This seems to happen when the whole robot is running behind.
        // It's not that harmful, it just means we don't extrapolate.
        if (dt > 0.04) {
            dt = 0;
        }
        final double correctedYaw = yaw + rate * dt;
        final Rotation2d yawNWU = Rotation2d.fromRotations(correctedYaw);
        m_log_yaw.log(() -> yawNWU);
        return yawNWU;
    }

    @Override
    public double getYawRateNWU() {
        final double yawRateRad_S = Units.rotationsToRadians(m_gyro.getAngularVelocityYaw());
        m_log_yaw_rate.log(() -> yawRateRad_S);
        return yawRateRad_S;
    }

    /** Not latency-compensated. */
    @Override
    public Rotation2d getPitchNWU() {
        final Rotation2d pitchNWU = Rotation2d.fromRotations(m_gyro.getPitch());
        m_log_pitch.log(() -> pitchNWU);
        return pitchNWU;
    }

    /** Not latency-compensated. */
    @Override
    public Rotation2d getRollNWU() {
        final Rotation2d rollNWU = Rotation2d.fromRotations(m_gyro.getRoll());
        m_log_roll.log(() -> rollNWU);
        return rollNWU;
    }

    @Override
    public void periodic() {
        if (m_gyro.isCalibrating())
            System.out.println("Redux Gyro Calibrating ......");
        final CanandgyroFaults activeFaults = m_gyro.getActiveFaults();
        if (activeFaults.faultsValid())
            System.out.println("WARNING: Redux Gyro fault!");

    }
}