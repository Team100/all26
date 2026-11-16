package org.team100.lib.motor.rev;

import java.util.function.Supplier;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Configuration for one SparkBase motor */
public class RevConfigurator {
    private static final boolean ACTUALLY_CRASH = false;
    /**
     * The CAN report period for the built-in encoder.
     * This is longer than the equivalent CTRE number, maybe
     * it should be 10?
     */
    private static final int ENCODER_REPORT_PERIOD_MS = 20;

    private final SparkBase m_motor;
    private final NeutralMode100 m_neutral;
    private final MotorPhase m_phase;
    private final double m_statorCurrentLimit;
    private final PIDConstants m_pid;
    /**
     * Used for the Minion motor, see SparkMaxConfig.Presets.CTRE_Minion.
     * 
     * REV do not document the default for any other motor, so pass zero
     * to ignore this parameter.
     */
    private final double m_commutationDegrees;
    /**
     * Number of samples to average for velocity measurement. Default is 64.
     * 
     * Position control should use more averaging, velocity control should use less.
     * 
     * The default is probably good for slow positional control.
     * 
     * For fast velocity control, try 2.
     * 
     * Pass zero to get the default.
     */
    private final int m_averageDepth;
    /**
     * Velocity is computed as the difference in position across this interval, in
     * milliseconds. Default is 100.
     * 
     * Position control should use a longer period, velocity control should use
     * less.
     * 
     * The default is probably good for slow positional control.
     * 
     * For fast velocity control, try 4.
     *
     * Pass zero to get the default.
     */
    private final int m_measurementPeriod;

    /**
     * Does not support supply limit.
     * 
     * @param averageDepth      for Max, must be in [1,64], default 64
     *                          for Flex, must be one of 1,2,4, or 8, default 8
     * @param measurementPeriod for Max, must be in [1,100], default 100
     *                          for Flex, must be in [8, 64], default 32
     */
    public RevConfigurator(
            LoggerFactory log,
            SparkBase motor,
            NeutralMode100 neutral,
            MotorPhase phase,
            CurrentLimit limit,
            PIDConstants pid,
            double commutationDegrees,
            int averageDepth,
            int measurementPeriod) {
        m_motor = motor;
        m_neutral = neutral;
        m_phase = phase;
        m_statorCurrentLimit =  limit.stator();
        m_pid = pid;
        // reapply the pid parameters if any change.
        m_pid.register(this::pidConfig);
        m_commutationDegrees = commutationDegrees;
        m_averageDepth = averageDepth;
        m_measurementPeriod = measurementPeriod;
    }

    /**
     * Makes config synchronous so we can see the errors
     */
    public void longCANTimeout() {
        crash(() -> m_motor.setCANTimeout(500));
    }

    /**
     * Makes everything asynchronous.
     * NOTE: this makes error-checking not work at all.
     */
    public void zeroCANTimeout() {
        crash(() -> m_motor.setCANTimeout(0));
    }

    /**
     * This is like the old resetFactoryDefaults() method.
     * This is the *only* place that "kResetSafeParameters" should be used.
     */
    public void resetDefaults() {
        SparkMaxConfig conf = new SparkMaxConfig();
        crash(() -> m_motor.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    public void baseConfig() {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
        conf.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
        conf.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        conf.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    /**
     * Configure velocity measurement.
     * 
     * Velocity control wants less filtering (less latency).
     * Position control wants more filtering (more accuracy).
     * 
     * @param isFlex Max uses "uvw" and Flex uses "quadrature"
     */
    public void velocityConfig(boolean isFlex) {
        SparkMaxConfig conf = new SparkMaxConfig();
        if (m_averageDepth != 0) {
            if (isFlex) {
                // TODO: verify that this works
                conf.encoder.quadratureAverageDepth(m_averageDepth);
            } else {
                conf.encoder.quadratureAverageDepth(m_averageDepth);
            }
        }
        if (m_measurementPeriod != 0) {
            if (isFlex) {
                // TODO: verify that this works
                conf.encoder.quadratureMeasurementPeriod(m_measurementPeriod);
            } else {
                conf.encoder.quadratureMeasurementPeriod(m_measurementPeriod);
            }
        }

        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    public void motorConfig() {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.idleMode(switch (m_neutral) {
            case COAST -> IdleMode.kCoast;
            case BRAKE -> IdleMode.kBrake;
        });
        conf.inverted(switch (m_phase) {
            case FORWARD -> false;
            case REVERSE -> true;
        });

        // Minion motor uses different commutation angle.
        if (m_commutationDegrees != 0) {
            conf.advanceCommutation(m_commutationDegrees);
        }

        conf.signals.primaryEncoderVelocityPeriodMs(ENCODER_REPORT_PERIOD_MS);
        conf.signals.primaryEncoderVelocityAlwaysOn(true);
        conf.signals.primaryEncoderPositionPeriodMs(ENCODER_REPORT_PERIOD_MS);
        conf.signals.primaryEncoderPositionAlwaysOn(true);
        // slower than default of 10; also affects things like motor temperature and
        // applied output.
        conf.signals.limitsPeriodMs(20);
        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    public void currentConfig() {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.smartCurrentLimit((int) m_statorCurrentLimit);
        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    /**
     * Changes the stator limit. This is useful for "holding torque" which might be
     * less than "grabbing torque".
     */
    public void overrideStatorLimit(int limit) {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.smartCurrentLimit(limit);
        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    /**
     * Returns the current limit to the initial setup.
     */
    public void endCurrentLimitOverride() {
        currentConfig();
    }

    /**
     * The REV PID units are as follows:
     * 
     * position
     * P = duty cycle per rev, start with 1.
     * I = duty cycle per rev * ms
     * D = duty cycle per rev/ms
     * 
     * velocity
     * P = duty cycle per RPM, start with 0.0002; rev example is 0.00005
     * I = duty cycle per RPM*ms
     * D = duty cycle per RPM/ms
     * 
     * @see https://docs.revrobotics.com/revlib/spark/closed-loop/units
     * @see https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
     */
    public void pidConfig() {
        double supplyVoltage = 12.0;
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.closedLoop.positionWrappingEnabled(false); // don't use position control
        conf.closedLoop.p(2 * Math.PI * m_pid.getPositionPV_Rad() / supplyVoltage, ClosedLoopSlot.kSlot0);
        conf.closedLoop.i(2 * Math.PI * m_pid.getPositionIV_RadS() / (1000 * supplyVoltage), ClosedLoopSlot.kSlot0);
        conf.closedLoop.d(2 * Math.PI * 1000 * m_pid.getPositionDVS_Rad() / supplyVoltage, ClosedLoopSlot.kSlot0);

        conf.closedLoop.p(2 * Math.PI * m_pid.getVelocityPVS_Rad() / (60 * supplyVoltage), ClosedLoopSlot.kSlot1);
        conf.closedLoop.i(2 * Math.PI * m_pid.getVelocityIVolt_Rad() / (60 * 1000 * supplyVoltage),
                ClosedLoopSlot.kSlot1);
        conf.closedLoop.d(2 * Math.PI * 1000 * m_pid.getVelocityDVS2_Rad() / (60 * supplyVoltage),
                ClosedLoopSlot.kSlot1);
        // We don't use wind-up control.
        conf.closedLoop.iZone(0, ClosedLoopSlot.kSlot0);
        conf.closedLoop.iZone(0, ClosedLoopSlot.kSlot1);
        // We don't use this type of feedforward at all, we use "arbitrary" feedforward.
        conf.closedLoop.apply(new FeedForwardConfig().kV(0, ClosedLoopSlot.kSlot0));
        conf.closedLoop.apply(new FeedForwardConfig().kV(0, ClosedLoopSlot.kSlot1));
        // Maximum output range.
        conf.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot0);
        conf.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    private void crash(Supplier<REVLibError> s) {
        REVLibError errorCode = s.get();
        if (errorCode != REVLibError.kOk) {
            if (ACTUALLY_CRASH)
                throw new IllegalStateException(errorCode.name());
            System.out.println("WARNING: ******************************************************");
            System.out.printf("WARNING: ****** Motor ID %d\n", m_motor.getDeviceId());
            System.out.println("WARNING: ****** MOTOR CONFIG HAS FAILED MOTOR IS NOT SET CORRECTLY ******");
            System.out.println("WARNING: " + errorCode.toString());
        }
    }
}
