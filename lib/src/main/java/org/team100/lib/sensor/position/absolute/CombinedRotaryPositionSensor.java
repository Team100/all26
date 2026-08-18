package org.team100.lib.sensor.position.absolute;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

import edu.wpi.first.math.MathUtil;

/**
 * Proxies an absolute sensor and an incremental sensor.
 * 
 * Synchronizes the incremental one to the absolute one, using a separate thread
 * with a delay of a few seconds.
 * 
 * Why delay? Because the RoboRIO duty-cycle input seems to return
 * slightly-wrong values initially.
 * 
 * The use case is absolute + incremental encoders, in order to do outboard
 * closed-loop position control with only outboard incremental encoders --
 * RoboRIO-attached absolute encoders are the primary, and the incremental
 * encoders are the secondary. Note in this case the "primary" absolute
 * measurement is [-pi,pi] but the "secondary" measurement is "unwrapped" i.e.
 * the domain is [-inf,inf].
 * 
 * The secondary is a RotaryMechanism, instead of an encoder, because we want
 * the *gear reduction* to be applied to the underlying encoder.
 */
public class CombinedRotaryPositionSensor implements RotaryPositionSensor {
    private final RotaryPositionSensor m_absolute;
    private final ProxyRotaryPositionSensor m_incremental;
    private final DoubleLogger m_log_absolute;
    private final DoubleLogger m_log_incremental;
    private final DoubleLogger m_log_incremental_wrapped;
    private final DoubleLogger m_log_combined;
    // for synchronization one-shot delayed task
    private final ScheduledExecutorService m_synchronizer;

    private boolean m_synchronized;

    /**
     * "Zeros" the incremental sensor.
     * 
     * @param absolute    absolute sensor wired to the RoboRIO
     * @param incremental incremental sensor that needs to be "zeroed"
     */
    public CombinedRotaryPositionSensor(
            LoggerFactory parent,
            RotaryPositionSensor absolute,
            ProxyRotaryPositionSensor incremental) {
        LoggerFactory log = parent.type(this);
        m_absolute = absolute;
        m_incremental = incremental;
        m_log_absolute = log.doubleLogger(Level.DEBUG, "absolute (rad))");
        m_log_incremental = log.doubleLogger(Level.TRACE, "incremental (rad)");
        m_log_incremental_wrapped = log.doubleLogger(Level.TRACE, "incremental wrapped (rad)");
        m_log_combined = log.doubleLogger(Level.DEBUG, "combined (rad)");
        // the duty cycle encoder seems to produce slightly-wrong values immediately
        // upon startup, so wait a bit before doing the synchronization
        m_synchronized = false;
        m_synchronizer = Executors.newSingleThreadScheduledExecutor();
        m_synchronizer.schedule(this::sync, 3, TimeUnit.SECONDS);
    }

    /**
     * Sync the absolute and incremental encoders.
     * 
     * This should only be called by the initializer executor, a few seconds after
     * startup, because the absolute encoder readings are wrong immediately after
     * startup.
     * 
     * Setting the encoder position is very slow, so just do it once.
     */
    void sync() {
        // sep 1 2025, joel removed this, i don't think we need it.
        // Cache.resetAll();
        // Assume the mechanism is stationary at startup, average a few measurements to
        // remove a little bit of noise.
        double sin = 0;
        double cos = 0;

        final int N = 10;
        for (int i = 0; i < N; ++i) {
            double pos = m_absolute.getWrappedPositionRad();
            cos += Math.cos(pos);
            sin += Math.sin(pos);

        }
        sin /= N;
        cos /= N;

        double absolutePosition = Math.atan2(sin, cos);

        m_incremental.setEncoderPosition(absolutePosition);
        m_synchronized = true;
    }

    public boolean isSynchronized() {
        return m_synchronized;
    }

    /**
     * Value is updated in Robot.robotPeriodic().
     * 
     * The secondary (incremental motor-integrated) measurement.
     */
    @Override
    public double getWrappedPositionRad() {
        return m_incremental.getWrappedPositionRad();
    }

    @Override
    public double getUnwrappedPositionRad() {
        return m_incremental.getUnwrappedPositionRad();
    }

    /**
     * Value is updated in Robot.robotPeriodic().
     * 
     * The secondary (incremental motor-integrated) measurement
     */
    @Override
    public double getVelocityRad_S() {
        return m_incremental.getVelocityRad_S();
    }

    @Override
    public void close() {
        m_absolute.close();
        m_incremental.close();
    }

    public void periodic() {
        m_absolute.periodic();
        m_incremental.periodic();
        m_log_absolute.log(m_absolute::getWrappedPositionRad);
        m_log_incremental.log(m_incremental::getWrappedPositionRad);
        m_log_incremental_wrapped.log(() -> MathUtil.angleModulus(m_incremental.getWrappedPositionRad()));
        m_log_combined.log(this::getWrappedPositionRad);
    }

    /**
     * The position as read by the absolute sensor. This is useful for debugging and
     * for the lash observer. Don't use this unless you know what you're doing.
     */
    double getAbsolutePositionRad() {
        return m_absolute.getWrappedPositionRad();
    }
}