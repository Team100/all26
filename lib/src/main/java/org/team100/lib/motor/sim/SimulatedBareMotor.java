package org.team100.lib.motor.sim;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.ModelR1Logger;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.sensor.position.incremental.sim.SimulatedBareEncoder;
import org.team100.lib.state.ModelR1;
import org.team100.lib.util.Math100;
import org.wpilib.driverstation.RobotState;

/**
 * Relies on Cache and Takt, so you must put Cache.refresh() and Takt.update()
 * in
 * Robot.robotPeriodic().
 */
public class SimulatedBareMotor implements BareMotor {
    private static final boolean DEBUG = false;

    private final double m_freeSpeedRad_S;

    private final LoggerFactory m_log;
    private final DoubleLogger m_log_duty;
    private final DoubleLogger m_log_velocityInput;
    private final DoubleLogger m_log_positionInput;
    private final DoubleLogger m_log_torqueInput;
    private final ModelR1Logger m_log_state;
    private final ObjectCache<ModelR1> m_stateCache;

    // just like in a real motor, the inputs remain until zeroed by the watchdog.
    // nullable; only one (velocity or position) is used at a time.
    private Double m_velocityInput;
    private Double m_positionInput;
    private Double m_torqueInput;

    private ModelR1 m_state = new ModelR1();

    private double m_time = Takt.get();

    public SimulatedBareMotor(LoggerFactory parent, double freeSpeedRad_S) {
        m_log = parent.type(this);
        m_freeSpeedRad_S = freeSpeedRad_S;
        m_log_duty = m_log.doubleLogger(Level.DEBUG, "duty_cycle");
        m_log_velocityInput = m_log.doubleLogger(Level.DEBUG, "velocity input");
        m_log_positionInput = m_log.doubleLogger(Level.DEBUG, "position input");
        m_log_torqueInput = m_log.doubleLogger(Level.DEBUG, "torque input");
        m_log_state = m_log.ModelR1Logger(Level.DEBUG, "state");
        m_stateCache = Cache.of(this::update);
    }

    private ModelR1 update() {
        // when disabled, motors don't keep moving.
        if (RobotState.isDisabled()) {
            m_velocityInput = 0.0;
            m_positionInput = null;
        }
        if (DEBUG) {
            System.out.printf("motor %s update\n", m_log.getRoot());
        }
        double dt = dt();
        if (DEBUG) {
            System.out.printf("SimulatedBareMotor dt %f\n", dt);
        }
        if (m_velocityInput != null) {
            if (DEBUG) {
                System.out.printf("SimulatedBareMotor v %f\n", m_velocityInput);
            }
            if (dt > 0.04) {
                // probably we should not extrapolate
                m_state = new ModelR1(m_state.x(), m_velocityInput);
            } else {
                m_state = new ModelR1(m_state.x() + m_velocityInput * dt, m_velocityInput);
            }
        }
        if (m_positionInput != null) {
            if (DEBUG) {
                System.out.printf("SimulatedBareMotor x %f\n", m_positionInput);
            }
            if (dt < 0.01) {
                // probably we should not differentiate
                m_state = new ModelR1(m_positionInput, m_state.v());
            } else {
                m_state = new ModelR1(m_positionInput, (m_positionInput - m_state.x()) / dt);
            }
        }
        if (DEBUG) {
            System.out.printf("SimulatedBareMotor state %s\n", m_state);
        }
        m_log_state.log(() -> m_state);
        return m_state;
    }

    double dt() {
        double now = Takt.get();
        double dt = now - m_time;
        m_time = now;
        return dt;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        final double output = Math.clamp(
                Math100.notNaN(dutyCycle), -1, 1);
        m_log_duty.log(() -> output);
        setVelocity(output * m_freeSpeedRad_S, 0);
    }

    @Override
    public void setVoltage(double volts) {
        setVelocity(volts * m_freeSpeedRad_S / 12, 0);
    }

    /** ignores accel and torque but logs them */
    @Override
    public void setVelocity(double velocityRad_S, double torqueNm) {
        if (DEBUG) {
            System.out.printf("motor %s set velocity %6.3f\n", m_log.getRoot(), velocityRad_S);
        }
        m_velocityInput = Math.clamp(
                Math100.notNaN(velocityRad_S), -m_freeSpeedRad_S, m_freeSpeedRad_S);
        m_torqueInput = torqueNm;
        // you can't use velocity and position control at the same time
        m_positionInput = null;
    }

    /** ignores velocity and torque */
    @Override
    public void setUnwrappedPosition(double position, double velocity, double torque) {
        if (DEBUG) {
            System.out.printf("motor %s set position %6.3f\n", m_log.getRoot(), position);
        }
        m_positionInput = Math100.notNaN(position);
        // you can't use velocity and position control at the same time
        m_velocityInput = null;
        m_torqueInput = null;
    }

    /** placeholder */
    @Override
    public double kROhms() {
        return 0.1;
    }

    /** placeholder */
    @Override
    public double kTNm_amp() {
        return 0.02;
    }

    @Override
    public double kFreeSpeedRPM() {
        return 6000;
    }

    @Override
    public IncrementalBareEncoder encoder() {
        return new SimulatedBareEncoder(m_log, this);
    }

    @Override
    public void stop() {
        m_velocityInput = 0.0;
    }

    @Override
    public void close() {
        //
    }

    @Override
    public double getVelocityRad_S() {
        return m_stateCache.get().v();
    }

    @Override
    public double getCurrent() {
        // this is totally wrong
        return getVelocityRad_S() / 10.0;
    }

    @Override
    public double getSupplyCurrent() {
        // no current measurement
        return 0;
    }

    @Override
    public double getUnwrappedPositionRad() {
        double pos = m_stateCache.get().x();
        if (Double.isNaN(pos))
            throw new IllegalArgumentException("motor pos");
        return pos;
    }

    /** resets the caches, so the new value is immediately available. */
    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        if (Double.isNaN(positionRad))
            throw new IllegalArgumentException("motor set position");
        m_positionInput = positionRad;
        m_stateCache.reset();
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        //
    }

    @Override
    public void periodic() {
        if (m_positionInput != null)
            m_log_positionInput.log(() -> m_positionInput);
        if (m_velocityInput != null)
            m_log_velocityInput.log(() -> m_velocityInput);
        if (m_torqueInput != null)
            m_log_torqueInput.log(() -> m_torqueInput);

    }

    /** resets the caches, so the new value is immediately available. */
    public void reset() {
        m_positionInput = 0.0;
        m_velocityInput = 0.0;
        m_time = Takt.get();
        m_stateCache.reset();
    }

    @Override
    public void play(double freq) {
    }
}
