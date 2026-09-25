package org.team100.lib.motor.sim;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.StateR1Logger;
import org.team100.lib.motor.Motor;
import org.team100.lib.state.StateR1;
import org.team100.lib.util.LowPassDerivative;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;

/**
 * Relies on Cache and Takt, so you must put Cache.refresh() and Takt.update()
 * in
 * Robot.robotPeriodic().
 */
public class SimulatedMotor implements Motor {
    private static final boolean DEBUG = false;

    private final double m_freeSpeedRad_S;

    private final LoggerFactory m_log;
    private final SimulatedEncoder m_encoder;
    private final DoubleLogger m_log_duty;
    private final DoubleLogger m_log_velocityInput;
    private final DoubleLogger m_log_positionInput;
    private final DoubleLogger m_log_torqueInput;
    private final StateR1Logger m_log_state;
    /** Ground-truth state. */
    final ObjectCache<StateR1> m_stateCache;
    final LowPassDerivative m_smoothDerivative;

    // Just like in a real motor, the inputs remain until zeroed by the watchdog.
    // Nullable; only one (velocity or position) is used at a time.
    private Double m_velocityInput;
    /** Ground-truth position input, with offset applied */
    private Double m_positionInput;
    private Double m_torqueInput;
    /** Ground-truth state. */
    private StateR1 m_state = new StateR1();

    public SimulatedMotor(LoggerFactory parent, double freeSpeedRad_S) {
        m_log = parent.type(this);
        m_freeSpeedRad_S = freeSpeedRad_S;
        m_encoder = new SimulatedEncoder(m_log, this);
        m_log_duty = m_log.doubleLogger(Level.DEBUG, "duty_cycle");
        m_log_velocityInput = m_log.doubleLogger(Level.DEBUG, "velocity input");
        m_log_positionInput = m_log.doubleLogger(Level.DEBUG, "position input");
        m_log_torqueInput = m_log.doubleLogger(Level.DEBUG, "torque input");
        m_log_state = m_log.StateR1Logger(Level.DEBUG, "state");
        m_stateCache = Cache.of(this::update);
        m_smoothDerivative = new LowPassDerivative();
        LogPoller.register(this::log);
    }

    private StateR1 update() {
        // when disabled, motors don't keep moving.
        if (RobotState.isDisabled()) {
            m_velocityInput = 0.0;
            m_positionInput = null;
        }
        if (DEBUG) {
            System.out.printf("motor %s update\n", m_log.getRoot());
        }
        double dt = TimedRobot100.LOOP_PERIOD_S;

        if (m_velocityInput != null) {
            if (DEBUG) {
                System.out.printf("SimulatedBareMotor v %f\n", m_velocityInput);
            }
            m_state = new StateR1(m_state.x() + m_velocityInput * dt, m_velocityInput);
        }
        if (m_positionInput != null) {
            if (DEBUG) {
                System.out.printf("SimulatedBareMotor x %f\n", m_positionInput);
            }
            m_state = new StateR1(m_positionInput, (m_positionInput - m_state.x()) / dt);
        }
        if (DEBUG) {
            System.out.printf("SimulatedBareMotor state %s\n", m_state);
        }
        m_log_state.log(() -> m_state);
        m_smoothDerivative.calculate(m_state.v());
        return m_state;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        final double output = MathUtil.clamp(
                Math100.notNaN(dutyCycle), -1, 1);
        m_log_duty.log(() -> output);
        setVelocity(output * m_freeSpeedRad_S, 0);
    }

    @Override
    public void setVoltage(double volts) {
        setVelocity(volts * m_freeSpeedRad_S / 12, 0);
    }

    @Override
    public void setCurrent(double current) {
        // NOTE: this is a ridiculous hack.
        if (m_velocityInput == null) {
            setVelocity(0.1, 0);
        } else {
            setVelocity(m_velocityInput + current / 100, current);
        }
        m_torqueInput = 0.0;
        m_positionInput = null;
    }

    /** ignores accel and torque but logs them */
    @Override
    public void setVelocity(double velocityRad_S, double torqueNm) {
        if (DEBUG) {
            System.out.printf("motor %s set velocity %6.3f\n", m_log.getRoot(), velocityRad_S);
        }
        m_velocityInput = MathUtil.clamp(
                Math100.notNaN(velocityRad_S), -m_freeSpeedRad_S, m_freeSpeedRad_S);
        m_torqueInput = torqueNm;
        // you can't use velocity and position control at the same time
        m_positionInput = null;
    }

    /**
     * Set the motor position, immediately, applying the encoder offset.
     * Ignores velocity and torque
     */
    @Override
    public void setUnwrappedPosition(double position, double velocity, double torque) {
        position += m_encoder.m_offset;
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
    public double R() {
        return 0.1;
    }

    /** placeholder */
    @Override
    public double kT() {
        return 0.02;
    }

    @Override
    public double kE() {
        return 12 / m_freeSpeedRad_S;
    }

    @Override
    public SimulatedEncoder encoder() {
        return m_encoder;
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
    public double getStatorCurrent() {
        // this is totally wrong
        return m_stateCache.get().v() / 10.0;
    }

    @Override
    public double getSupplyCurrent() {
        // no current measurement
        return 0;
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        //
    }

    private void log() {
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
        m_stateCache.reset();
    }

    @Override
    public void play(double freq) {
    }
}
