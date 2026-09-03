package org.team100.lib.motor.sim;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.StateR1Logger;
import org.team100.lib.motor.Motor;
import org.team100.lib.sensor.position.incremental.sim.SimulatedEncoder;
import org.team100.lib.state.StateR1;
import org.team100.lib.util.LowPassDerivative;
import org.team100.lib.util.Math100;
import org.wpilib.driverstation.RobotState;

/**
 * Relies on Cache and Takt, so you must put Cache.refresh() and Takt.update()
 * in
 * Robot.robotPeriodic().
 */
public class SimulatedMotor implements Motor {
    private static final boolean DEBUG = false;

    private final double m_freeSpeedRad_S;

    private final LoggerFactory m_log;
    private final DoubleLogger m_log_duty;
    private final DoubleLogger m_log_velocityInput;
    private final DoubleLogger m_log_positionInput;
    private final DoubleLogger m_log_torqueInput;
    private final StateR1Logger m_log_state;
    private final DoubleLogger m_log_unwrapped_position;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_accel;
    private final ObjectCache<StateR1> m_stateCache;
    private final LowPassDerivative m_smoothDerivative;

    // just like in a real motor, the inputs remain until zeroed by the watchdog.
    // nullable; only one (velocity or position) is used at a time.
    private Double m_velocityInput;
    private Double m_positionInput;
    private Double m_torqueInput;

    private StateR1 m_state = new StateR1();

    public SimulatedMotor(LoggerFactory parent, double freeSpeedRad_S) {
        m_log = parent.type(this);
        m_freeSpeedRad_S = freeSpeedRad_S;
        m_log_duty = m_log.doubleLogger(Level.DEBUG, "duty_cycle");
        m_log_velocityInput = m_log.doubleLogger(Level.DEBUG, "velocity input");
        m_log_positionInput = m_log.doubleLogger(Level.DEBUG, "position input");
        m_log_torqueInput = m_log.doubleLogger(Level.DEBUG, "torque input");
        m_log_state = m_log.StateR1Logger(Level.DEBUG, "state");
        m_log_unwrapped_position = m_log.doubleLogger(Level.DEBUG, "unwrapped position (rad)");
        m_log_velocity = m_log.doubleLogger(Level.DEBUG, "velocity (rad_s)");
        m_log_accel = m_log.doubleLogger(Level.DEBUG, "accel (rad_s2)");
        m_stateCache = Cache.of(this::update);
        m_smoothDerivative = new LowPassDerivative();
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
        final double output = Math.clamp(
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
        // TODO: a better simulated current control
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
        return new SimulatedEncoder(m_log, this);
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
    public double getAccelerationRad_S2() {
        // this is computed in update
        return m_smoothDerivative.lastValue();
    }

    @Override
    public double getStatorCurrent() {
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

    /**
     * Set the state directly. Also zeros velocity; if you reset the
     * position while in motion you shouldn't expect it to work anyway.
     * 
     * resets the caches, so the new value is immediately available.
     */
    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        if (Double.isNaN(positionRad))
            throw new IllegalArgumentException("motor set position");
        m_state = new StateR1(positionRad, 0);
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
        m_log_unwrapped_position.log(this::getUnwrappedPositionRad);
        m_log_velocity.log(this::getVelocityRad_S);
        m_log_accel.log(this::getAccelerationRad_S2);
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
