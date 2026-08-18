package org.team100.lib.motor.ctre;

import java.util.function.Supplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.DoubleCache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.sensor.position.incremental.ctre.Talon6Encoder;
import org.team100.lib.util.CanId;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * Superclass for TalonFX motors.
 * 
 * Relies on Cache and Takt, so you must put Cache.refresh() and Takt.update()
 * in
 * Robot.robotPeriodic().
 */
public abstract class Talon6Motor implements BareMotor {
    private final LoggerFactory m_log;
    private static final boolean DEBUG = false;

    private final TalonFX m_motor;
    private final PhoenixConfigurator m_configurator;
    private final Friction m_friction;

    // CACHES
    // Two levels of caching here: the cotemporal cache caches the value
    // and also the supplier

    /** radians, latency-compensated. */
    protected final DoubleCache m_position;
    /** radians per second */
    protected final DoubleCache m_velocity;
    protected final DoubleCache m_dutyCycle;
    protected final DoubleCache m_error;
    protected final DoubleCache m_supplyCurrent;
    protected final DoubleCache m_supplyVoltage;
    protected final DoubleCache m_statorCurrent;
    protected final DoubleCache m_temp;

    /////////////////////////////////////
    // CONTROL REQUESTS
    //
    // caching the control requests saves allocation
    //
    private final VelocityVoltage m_velocityVoltage;
    private final DutyCycleOut m_dutyCycleOut;
    private final VoltageOut m_voltageOut;
    private final PositionVoltage m_positionVoltage;
    private final MusicTone m_music;

    // LOGGERS
    private final DoubleLogger m_log_desired_duty;
    private final DoubleLogger m_log_desired_voltage;
    /** rad */
    private final DoubleLogger m_log_desired_position;
    /** rad/s */
    private final DoubleLogger m_log_desired_speed;
    private final DoubleLogger m_log_friction_FF;
    private final DoubleLogger m_log_velocity_FF;
    private final DoubleLogger m_totalFeedForward;
    private final DoubleLogger m_log_torque_FF;
    /** rad */
    private final DoubleLogger m_log_position;
    /** rad/s */
    private final DoubleLogger m_log_velocity;
    /** duty cycle */
    private final DoubleLogger m_log_output;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_supply_current;
    private final DoubleLogger m_log_supplyVoltage;
    private final DoubleLogger m_log_stator_current;
    private final DoubleLogger m_log_temp;

    protected Talon6Motor(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CanId canId,
            NeutralMode100 neutral,
            MotorPhase motorPhase,
            CurrentLimit limit,
            Friction friction,
            PIDConstants pid) {
        currentLog.register(this);
        //////////////////////////////////////
        //
        // CONTROL REQUESTS
        //
        m_velocityVoltage = new VelocityVoltage(0);
        m_dutyCycleOut = new DutyCycleOut(0);
        m_voltageOut = new VoltageOut(0);
        m_positionVoltage = new PositionVoltage(0);
        m_music = new MusicTone(0);

        //////////////////////////////////////
        // Update frequencies.
        // make control synchronous, i.e. "actuate immediately." See
        // https://github.com/Team254/FRC-2024-Public/blob/040f653744c9b18182be5f6bc51a7e505e346e59/src/main/java/com/team254/lib/ctre/swerve/SwerveModule.java#L210
        m_velocityVoltage.UpdateFreqHz = 0;
        m_dutyCycleOut.UpdateFreqHz = 0;
        m_voltageOut.UpdateFreqHz = 0;
        m_positionVoltage.UpdateFreqHz = 0;

        m_log = parent.type(this);
        m_motor = new TalonFX(canId.id);
        m_friction = friction;

        m_configurator = new PhoenixConfigurator(
                m_motor,
                neutral,
                motorPhase,
                limit,
                pid);
        m_configurator.logCrashStatus();
        m_configurator.baseConfig();
        m_configurator.motorConfig();
        m_configurator.currentConfig();
        m_configurator.pidConfig();
        m_configurator.audioConfig();

        // Cache the status signal getters.
        StatusSignal<Angle> motorPositionRev = m_motor.getPosition();
        StatusSignal<AngularVelocity> motorVelocityRev_S = m_motor.getVelocity();
        StatusSignal<Double> motorDutyCycle = m_motor.getDutyCycle();
        StatusSignal<Double> motorClosedLoopError = m_motor.getClosedLoopError();
        StatusSignal<Current> motorSupplyCurrent = m_motor.getSupplyCurrent();
        StatusSignal<Voltage> motorSupplyVoltage = m_motor.getSupplyVoltage();
        StatusSignal<Current> motorStatorCurrent = m_motor.getStatorCurrent();
        StatusSignal<Temperature> motorDeviceTemp = m_motor.getDeviceTemp();

        // The memoizer refreshes all the signals at once.
        Cache.registerSignal(motorPositionRev);
        Cache.registerSignal(motorVelocityRev_S);
        Cache.registerSignal(motorDutyCycle);
        Cache.registerSignal(motorClosedLoopError);
        Cache.registerSignal(motorSupplyCurrent);
        Cache.registerSignal(motorSupplyVoltage);
        Cache.registerSignal(motorStatorCurrent);
        Cache.registerSignal(motorDeviceTemp);

        // None of these need to refresh.
        // this latency compensation uses takt time rather than the real clock.
        m_position = Cache.ofDouble(() -> {
            double latency = Utils.fpgaToCurrentTime(Takt.get()) - motorPositionRev.getTimestamp().getTime();
            if (latency > 0.04) {
                if (DEBUG)
                    System.out.printf("WARNING: stale position %s latency %f ", canId, latency);
                latency = 0.1;
            }
            double motorRad = motorPositionRev.getValueAsDouble() * 2 * Math.PI;
            double motorRad_S = motorVelocityRev_S.getValueAsDouble() * 2 * Math.PI;
            return motorRad + (motorRad_S * latency);
        });
        m_velocity = Cache.ofDouble(() -> motorVelocityRev_S.getValueAsDouble() * 2 * Math.PI);
        m_dutyCycle = Cache.ofDouble(() -> motorDutyCycle.getValueAsDouble());
        m_error = Cache.ofDouble(() -> motorClosedLoopError.getValueAsDouble());
        m_supplyCurrent = Cache.ofDouble(() -> motorSupplyCurrent.getValueAsDouble());
        m_supplyVoltage = Cache.ofDouble(() -> motorSupplyVoltage.getValueAsDouble());
        m_statorCurrent = Cache.ofDouble(() -> motorStatorCurrent.getValueAsDouble());
        m_temp = Cache.ofDouble(() -> motorDeviceTemp.getValueAsDouble());

        m_log_desired_duty = m_log.doubleLogger(Level.DEBUG, "desired duty cycle [-1,1]");
        m_log_desired_voltage = m_log.doubleLogger(Level.DEBUG, "desired voltage (V)");
        m_log_desired_position = m_log.doubleLogger(Level.DEBUG, "desired position (rad)");
        m_log_desired_speed = m_log.doubleLogger(Level.DEBUG, "desired speed (rad_s)");
        m_log_friction_FF = m_log.doubleLogger(Level.TRACE, "friction feedforward (V)");
        m_log_velocity_FF = m_log.doubleLogger(Level.TRACE, "velocity feedforward (V)");
        m_log_torque_FF = m_log.doubleLogger(Level.TRACE, "torque feedforward (V)");
        m_totalFeedForward = m_log.doubleLogger(Level.TRACE, "total feedforward (V)");

        m_log_position = m_log.doubleLogger(Level.DEBUG, "position (rad)");
        m_log_velocity = m_log.doubleLogger(Level.COMP, "velocity (rad_s)");
        m_log_output = m_log.doubleLogger(Level.COMP, "output [-1,1]");
        m_log_error = m_log.doubleLogger(Level.TRACE, "error");
        m_log_supply_current = m_log.doubleLogger(Level.DEBUG, "supply current (A)");
        m_log_supplyVoltage = m_log.doubleLogger(Level.DEBUG, "supply voltage (V)");
        m_log_stator_current = m_log.doubleLogger(Level.DEBUG, "stator current (A)");
        m_log_temp = m_log.doubleLogger(Level.DEBUG, "temperature (C)");

        m_log.intLogger(Level.TRACE, "Device ID").log(() -> canId.id);
    }

    /** Set duty cycle immediately. */
    @Override
    public void setDutyCycle(double output) {
        warn(() -> m_motor.setControl(m_dutyCycleOut.withOutput(output)));
        m_log_desired_duty.log(() -> output);
    }

    @Override
    public void setVoltage(double volts) {
        warn(() -> m_motor.setControl(m_voltageOut.withOutput(volts)));
        m_log_desired_voltage.log(() -> volts);
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        int currentA = (int) (torqueNm / kTNm_amp());
        m_configurator.overrideStatorLimit(currentA);
    }

    @Override
    public double getCurrent() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getSupplyCurrent() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }

    /**
     * Use VelocityVoltage outboard PID control to hold the given velocity, with
     * friction, velocity, acceleration, and torque feedforwards.
     * 
     * Actuates immediately.
     * 
     * Previously there was an accel term. Accel should be handled using
     * Subsystem-level dynamics.
     */
    @Override
    public void setVelocity(double motorRad_S, double torqueNm) {
        double backEMFVolts = backEMFVoltage(motorRad_S);
        double frictionFFVolts = m_friction.frictionFFVolts(motorRad_S);
        double torqueFFVolts = getTorqueFFVolts(torqueNm);
        double FFVolts = backEMFVolts + frictionFFVolts + torqueFFVolts;

        // CTRE control unit is rev/s.
        warn(() -> m_motor.setControl(
                m_velocityVoltage
                        .withSlot(1)
                        .withVelocity(motorRad_S / (2 * Math.PI))
                        .withFeedForward(FFVolts)));

        m_log_desired_speed.log(() -> motorRad_S);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> backEMFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
        m_totalFeedForward.log(() -> FFVolts);
    }

    @Override
    public void play(double freq) {
        m_motor.setControl(m_music.withAudioFrequency(freq));
    }

    /**
     * Use PositionVoltage outboard PID control to hold the given position, with
     * friction, velocity, accel, and torque feedforwards.
     * 
     * Actuates immediately.
     * 
     * Motor revolutions wind up, so setting 0 rad and 2pi rad are different.
     * 
     * Previously there was an accel term. Accel should be handled using
     * Subsystem-level dynamics.
     */
    @Override
    public void setUnwrappedPosition(
            double motorRad,
            double motorRad_S,
            double torqueNm) {
        double backEMFVolts = backEMFVoltage(motorRad_S);
        double frictionFFVolts = m_friction.frictionFFVolts(motorRad_S);
        double torqueFFVolts = getTorqueFFVolts(torqueNm);
        double FFVolts = backEMFVolts + frictionFFVolts + torqueFFVolts;

        // CTRE control unit is rev.
        warn(() -> m_motor.setControl(
                m_positionVoltage
                        .withSlot(0)
                        .withPosition(motorRad / (2 * Math.PI))
                        .withFeedForward(FFVolts)));

        m_log_desired_position.log(() -> motorRad);
        m_log_desired_speed.log(() -> motorRad_S);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> backEMFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
        m_totalFeedForward.log(() -> FFVolts);
    }

    /** Not latency-compensated. Updated in Robot.robotPeriodic(). */
    @Override
    public double getVelocityRad_S() {
        return m_velocity.getAsDouble();
    }

    @Override
    public Talon6Encoder encoder() {
        return new Talon6Encoder(m_log, this);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void reset() {
        m_position.reset();
        m_velocity.reset();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    /**
     * Sets integrated sensor position to zero.
     * 
     * Note this takes **FOREVER**, like tens of milliseconds, so you can only do it
     * at startup.
     * 
     * TODO: I think this is unnecessary, so I'm commenting it out.
     */
    // public void resetEncoderPosition() {
    // System.out.println("WARNING: Setting CTRE encoder position is very slow!");
    // warn(() -> m_motor.setPosition(0, 1));
    // m_position.reset();
    // m_velocity.reset();
    // }

    /**
     * Set integrated sensor position in radians.
     * 
     * This is the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi.
     * 
     * Note this takes **FOREVER**, like tens of milliseconds, so you can only do it
     * at startup.
     */
    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        System.out.println("WARNING: Setting CTRE encoder position is very slow!");
        warn(() -> m_motor.setPosition(positionRad / (2.0 * Math.PI), 1));
    }

    /**
     * This is the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi.
     * 
     * Latency-compensated, represents the current Takt.
     * Updated in `Robot.robotPeriodic()`.
     */
    @Override
    public double getUnwrappedPositionRad() {
        return m_position.getAsDouble();
    }

    @Override
    public void periodic() {
        log();
    }

    /////////////////////////////////////////////

    private void log() {
        m_log_position.log(m_position);
        m_log_velocity.log(m_velocity);
        m_log_output.log(m_dutyCycle);
        m_log_error.log(m_error);
        m_log_supply_current.log(m_supplyCurrent);
        m_log_supplyVoltage.log(m_supplyVoltage);
        m_log_stator_current.log(m_statorCurrent);
        m_log_temp.log(m_temp);
    }

    private static void warn(Supplier<StatusCode> s) {
        StatusCode statusCode = s.get();
        if (statusCode.isError()) {
            System.out.println("WARNING: " + statusCode.toString());
        }
    }
}
