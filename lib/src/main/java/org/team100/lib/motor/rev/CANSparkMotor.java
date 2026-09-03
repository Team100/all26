package org.team100.lib.motor.rev;

import java.util.function.Supplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.DoubleCache;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.sensor.position.incremental.rev.CANSparkEncoder;
import org.team100.lib.util.LowPassDerivative;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.ControlType;

/**
 * Base class for REV motors.
 * 
 * Relies on Cache and Takt, so you must put Cache.refresh() and Takt.update()
 * in
 * Robot.robotPeriodic().
 * 
 * Current limit is stator current.
 * 
 * Supply current is unmeasured and unlimited.
 * 
 * The REV closed-loop controller mechanism is used for position and velocity
 * control. Position control uses "slot zero" and velocity control uses "slot
 * one".
 * 
 * WARNING! REV motors are not good for velocity-controlled flywheels, because
 * the built-in encoder is noisy. The default filters induce much too much delay
 * to be useful; turning the filters all the way down helps.
 * 
 * WARNING! REV voltage control and current control ALSO use closed-loop SLOT
 * ZERO!!
 * 
 * https://www.chiefdelphi.com/t/psa-default-neo-sparkmax-velocity-readings-are-still-bad-for-flywheels/454453
 * https://www.chiefdelphi.com/t/psa-rev-spark-default-velocity-filtering-is-still-really-bad-for-flywheels/514567
 * https://www.chiefdelphi.com/t/shooter-encoder/400211/10
 * https://www.chiefdelphi.com/t/frc-4481-team-rembrandts-2026-build-thread-open-alliance/507296/475
 * https://docs.revrobotics.com/brushless/spark-max/gs/make-it-spin#limiting-current
 * https://www.chiefdelphi.com/t/rev-robotics-2024-2025/471083/26
 * https://www.reca.lc/flywheel
 */
public abstract class CANSparkMotor implements Motor {
    private final LoggerFactory m_log;
    private final Friction m_friction;
    private final SparkBase m_motor;
    private final RevConfigurator m_configurator;
    private final SparkLimitSwitch m_forLimitSwitch;
    private final SparkLimitSwitch m_revLimitSwitch;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_pidController;
    private final LowPassDerivative m_smoothDerivative;

    // CACHES

    /** radians */
    private final DoubleCache m_position;
    /** radians per second */
    private final DoubleCache m_velocity;
    /** radians per second squared */
    private final DoubleCache m_acceleration;
    /** amps */
    private final DoubleCache m_statorCurrent;
    /** volts */
    private final DoubleCache m_supplyVoltage;
    /** duty cycle */
    private final DoubleCache m_output;

    // LOGGERS
    /** rad */
    private final DoubleLogger m_log_desired_position;
    /** rad/s */
    private final DoubleLogger m_log_desired_speed;
    private final DoubleLogger m_log_friction_FF;
    private final DoubleLogger m_log_velocity_FF;
    private final DoubleLogger m_log_torque_FF;
    /** duty cycle */
    private final DoubleLogger m_log_output;
    private final DoubleLogger m_log_desired_voltage;
    private final DoubleLogger m_log_desired_current;
    /** rad */
    private final DoubleLogger m_log_position;
    /** rad/s */
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_accel;
    private final DoubleLogger m_log_stator_current;
    private final DoubleLogger m_log_supplyVoltage;

    protected CANSparkMotor(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            SparkBase motor,
            NeutralMode100 neutral,
            MotorPhase motorPhase,
            CurrentLimit limit,
            Friction friction,
            PIDConstants pid,
            double commutationDegrees,
            int averageDepth,
            int measurementPeriod,
            boolean isFlex) {
        currentLog.register(this);
        m_motor = motor;
        m_log = parent.type(this);
        m_friction = friction;

        m_configurator = new RevConfigurator(
                m_log,
                m_motor,
                neutral,
                motorPhase,
                limit,
                pid,
                commutationDegrees,
                averageDepth,
                measurementPeriod);
        m_configurator.longCANTimeout();
        m_configurator.resetDefaults();
        m_configurator.baseConfig();
        m_configurator.motorConfig();
        m_configurator.velocityConfig(isFlex);
        m_configurator.currentConfig();
        m_configurator.pidConfig();
        m_configurator.zeroCANTimeout();

        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getClosedLoopController();
        m_smoothDerivative = new LowPassDerivative();

        // LIMIT SWITCHES
        m_forLimitSwitch = m_motor.getForwardLimitSwitch();
        m_revLimitSwitch = m_motor.getReverseLimitSwitch();

        // CACHES
        // TODO: fix for 2027
        m_position = Cache.ofDouble(() -> m_encoder.getPosition().get() * 2 * Math.PI);
        m_velocity = Cache.ofDouble(() -> m_encoder.getVelocity().get() * 2 * Math.PI / 60);
        m_acceleration = Cache.ofDouble(() -> m_smoothDerivative.calculate(m_velocity.getAsDouble()));
        m_statorCurrent = Cache.ofDouble(() -> m_motor.getOutputCurrent().get());
        m_supplyVoltage = Cache.ofDouble(() -> m_motor.getBusVoltage().get());
        m_output = Cache.ofDouble(() -> m_motor.getAppliedOutput().get());

        // LOGGERS
        m_log_desired_position = m_log.doubleLogger(Level.DEBUG, "desired position (rad)");
        m_log_desired_speed = m_log.doubleLogger(Level.DEBUG, "desired speed (rad_s)");
        m_log_friction_FF = m_log.doubleLogger(Level.TRACE, "friction feedforward (V)");
        m_log_velocity_FF = m_log.doubleLogger(Level.TRACE, "velocity feedforward (V)");
        m_log_torque_FF = m_log.doubleLogger(Level.TRACE, "torque feedforward (V)");
        m_log_output = m_log.doubleLogger(Level.DEBUG, "output [-1,1]");
        m_log_desired_voltage = m_log.doubleLogger(Level.DEBUG, "desired voltage (V)");
        m_log_desired_current = m_log.doubleLogger(Level.DEBUG, "desired current (A)");
        m_log_position = m_log.doubleLogger(Level.DEBUG, "position (rad)");
        m_log_velocity = m_log.doubleLogger(Level.DEBUG, "velocity (rad_s)");
        m_log_accel = m_log.doubleLogger(Level.DEBUG, "accel (rad_s2)");
        m_log_stator_current = m_log.doubleLogger(Level.DEBUG, "stator current (A)");
        m_log_supplyVoltage = m_log.doubleLogger(Level.DEBUG, "voltage (V)");
        m_log.intLogger(Level.TRACE, "Device ID").log(m_motor::getDeviceId);
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.setThrottle(output);
        m_log_output.log(() -> output);
    }

    /**
     * Warning! This is a closed-loop control that uses PID SLOT ZERO!
     */
    @Override
    public void setVoltage(double volts) {
        m_pidController.setSetpoint(volts, ControlType.kVoltage);
        m_log_desired_voltage.log(() -> volts);
    }

    /**
     * Warning! This is a closed-loop control that uses PID SLOT ZERO!
     */
    @Override
    public void setCurrent(double amps) {
        m_pidController.setSetpoint(amps, ControlType.kCurrent);
        m_log_desired_current.log(() -> amps);
    }

    public boolean getForwardLimitSwitch() {
        return m_forLimitSwitch.isPressed().get();
    }

    public boolean getReverseLimitSwitch() {
        return m_revLimitSwitch.isPressed().get();
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        int currentA = (int) (torqueNm / kT());
        m_configurator.overrideStatorLimit(currentA);
    }

    /**
     * Use outboard PID control to hold the given velocity, with velocity,
     * acceleration, and torque feedforwards.
     */
    @Override
    public void setVelocity(double motorRad_S, double torqueNm) {
        double backEMFVolts = backEMFVoltage(motorRad_S);
        double frictionFFVolts = m_friction.frictionFFVolts(motorRad_S);
        double torqueFFVolts = getTorqueFFVolts(torqueNm);
        double FFVolts = backEMFVolts + frictionFFVolts + torqueFFVolts;

        // REV control unit is RPM
        warn(() -> m_pidController.setSetpoint(
                60 * motorRad_S / (2 * Math.PI),
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot1,
                FFVolts,
                ArbFFUnits.kVoltage));

        m_log_desired_speed.log(() -> motorRad_S);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> backEMFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
    }

    /**
     * Use outboard PID control to hold the given position, with velocity and torque
     * feedforwards.
     * 
     * Motor revolutions wind up, so setting 0 rad and 2pi rad are different.
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

        // REV control unit is revolutions
        warn(() -> m_pidController.setSetpoint(
                motorRad / (2 * Math.PI),
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                FFVolts,
                ArbFFUnits.kVoltage));

        m_log_desired_position.log(() -> motorRad);
        m_log_desired_speed.log(() -> motorRad_S);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> backEMFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
    }

    /** Value is updated in Robot.robotPeriodic(). */
    @Override
    public double getUnwrappedPositionRad() {
        return m_position.getAsDouble();
    }

    /** Value is updated in Robot.robotPeriodic(). */
    @Override
    public double getVelocityRad_S() {
        return m_velocity.getAsDouble();
    }

    /** Value is updated in Robot.robotPeriodic(). */
    @Override
    public double getAccelerationRad_S2() {
        return m_acceleration.getAsDouble();
    }

    @Override
    public double getStatorCurrent() {
        return m_statorCurrent.getAsDouble();
    }

    @Override
    public double getSupplyCurrent() {
        // NOTE: REV does not provide supply current.
        return 0;
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        warn(() -> m_encoder.setPosition(positionRad / (2.0 * Math.PI)));
    }

    @Override
    public CANSparkEncoder encoder() {
        return new CANSparkEncoder(m_log, this);
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
     */
    public void resetEncoderPosition() {
        warn(() -> m_encoder.setPosition(0));
        m_position.reset();
        m_velocity.reset();
    }

    @Override
    public void periodic() {
        log();
    }

    @Override
    public void play(double freq) {
    }

    ///////////////////////////////////////////

    private void log() {
        m_log_position.log(m_position);
        m_log_velocity.log(m_velocity);
        m_log_accel.log(m_acceleration);
        m_log_output.log(m_output);
        m_log_stator_current.log(m_statorCurrent);
        m_log_supplyVoltage.log(m_supplyVoltage);
    }

    private static void warn(Supplier<REVLibError> s) {
        REVLibError errorCode = s.get();
        if (errorCode != REVLibError.kOk) {
            System.out.println("WARNING: " + errorCode.name());
        }
    }
}
