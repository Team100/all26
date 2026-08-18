package org.team100.lib.sensor.gyro;

import java.util.Random;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.DoubleCache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A simulated gyro that uses drivetrain odometry.
 */
public class SimulatedGyro implements Gyro {
    private static final double SAMPLE_RATE = 100;
    /** White noise in rad/s */
    private static final double NOISE = 4e-4 * Math.sqrt(SAMPLE_RATE);
    private static final double BIAS_NOISE = 1e-5;
    private final Random m_rand;
    private final Rotation2dLogger m_log_yaw;
    private final DoubleLogger m_log_yaw_rate;
    private final SwerveKinodynamics m_kinodynamics;
    private final SwerveModuleCollection m_moduleCollection;
    private final double m_driftRateRad_S;
    private final DoubleCache m_headingCache;

    /** Actual heading, from odometry, plus drift. */
    private double m_heading;
    /** Time of most-recent sample, to compute dt. */
    private double m_time;

    public SimulatedGyro(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            SwerveModuleCollection collection,
            double driftRateRad_S) {
        LoggerFactory log = parent.type(this);
        m_rand = new Random();
        m_log_yaw = log.rotation2dLogger(Level.TRACE, "Yaw NWU (rad)");
        m_log_yaw_rate = log.doubleLogger(Level.TRACE, "Yaw Rate NWU (rad_s)");
        m_heading = 0;
        m_time = Takt.get();

        m_kinodynamics = kinodynamics;
        m_moduleCollection = collection;
        m_driftRateRad_S = driftRateRad_S;

        m_headingCache = Cache.ofDouble(this::update);
    }

    @Override
    public double white_noise() {
        return NOISE;
    }

    @Override
    public double bias_noise() {
        return BIAS_NOISE;
    }

    double update() {
        double dt = dt();
        if (dt > 0.04) {
            // clock is unreliable, ignore it
            dt = 0;
        }
        SwerveModuleStates states = m_moduleCollection.states();
        ChassisSpeeds speeds = m_kinodynamics.toChassisSpeedsWithDiscretization(states, 0.02);
        double noiseRad_S = NOISE * m_rand.nextGaussian();
        m_heading += (speeds.omegaRadiansPerSecond + m_driftRateRad_S + noiseRad_S) * dt;
        return m_heading;
    }

    double dt() {
        double now = Takt.get();
        double dt = now - m_time;
        m_time = now;
        return dt;
    }

    @Override
    public Rotation2d getYawNWU() {
        final Rotation2d yawNWU = new Rotation2d(m_headingCache.getAsDouble());
        m_log_yaw.log(() -> yawNWU);
        return yawNWU;
    }

    @Override
    public double getYawRateNWU() {
        SwerveModuleStates states = m_moduleCollection.states();
        ChassisSpeeds speeds = m_kinodynamics.toChassisSpeedsWithDiscretization(states, 0.02);
        double yawRateRad_S = speeds.omegaRadiansPerSecond;
        m_log_yaw_rate.log(() -> yawRateRad_S);
        return yawRateRad_S;
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
