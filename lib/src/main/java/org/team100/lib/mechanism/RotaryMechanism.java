package org.team100.lib.mechanism;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.music.Player;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.state.ModelR1;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;

/**
 * Uses a motor and gears to produce rotational output, e.g. an arm joint.
 * 
 * Motor velocity and accel is higher than mechanism, required torque is lower,
 * using the supplied gear ratio.
 * 
 * The position limits used to be enforced by a proxy, but now they're here: it
 * seems simpler that way.
 * 
 * Computes a noisy estimate of actual acceleration using backwards finite
 * difference.
 */
public class RotaryMechanism implements Player {
    private final BareMotor m_motor;
    private final RotaryPositionSensor m_sensor;
    private final double m_gearRatio;
    private final double m_minPositionRad;
    private final double m_maxPositionRad;
    private final DoubleLogger m_log_accel;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_desired_position;
    private final DoubleLogger m_log_filtered_velocity;
    LinearFilter lowPassFilter = LinearFilter.singlePoleIIR(0.2375, TimedRobot100.LOOP_PERIOD_S);

    /** For computing acceleration. */
    private double m_velocity;

    /**
     * The provided sensor encapsulates the motor sensor and/or the external
     * absolute sensor, if used. See ProxyRotaryPositionSensor and
     * CombinedRotaryPositionSensor.
     */
    public RotaryMechanism(
            LoggerFactory parent,
            BareMotor motor,
            RotaryPositionSensor sensor,
            double gearRatio,
            double minPositionRad,
            double maxPositionRad) {
        LoggerFactory log = parent.type(this);
        m_motor = motor;
        m_sensor = sensor;
        m_gearRatio = gearRatio;
        m_minPositionRad = minPositionRad;
        m_maxPositionRad = maxPositionRad;
        m_log_accel = log.doubleLogger(Level.DEBUG, "accel (rad_s2)");
        m_log_velocity = log.doubleLogger(Level.DEBUG, "velocity (rad_s)");
        m_log_position = log.doubleLogger(Level.DEBUG, "position (rad)");
        m_log_desired_position = log.doubleLogger(Level.DEBUG, "desired position (rad)");
        m_log_filtered_velocity = log.doubleLogger(Level.DEBUG, "filtered velocity (rad_s)");
    }

    /** There is no absolute position sensor in this case. */
    public RotaryMechanism(
            LoggerFactory parent,
            BareMotor motor,
            IncrementalBareEncoder encoder,
            double initialPosition,
            double gearRatio,
            double minPositionRad,
            double maxPositionRad) {
        this(parent, motor,
                new ProxyRotaryPositionSensor(encoder, gearRatio, initialPosition),
                gearRatio, minPositionRad, maxPositionRad);
    }

    /** Use for homing. */
    public void setDutyCycleUnlimited(double output) {
        m_motor.setDutyCycle(output);
    }

    /** Should actuate immediately. Enforces position limit using the encoder. */
    public void setDutyCycle(double output) {
        double posRad = getWrappedPositionRad();
        if (output < 0 && posRad < m_minPositionRad) {
            m_motor.stop();
            return;
        }
        if (output > 0 && posRad > m_maxPositionRad) {
            m_motor.stop();
            return;
        }
        m_motor.setDutyCycle(output);
    }

    /** For tuning friction. */
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    public void setTorqueLimit(double torqueNm) {
        m_motor.setTorqueLimit(torqueNm / m_gearRatio);
    }

    /**
     * Should actuate immediately. Use for homing.
     * 
     * Previously this included an accel term. Acceleration should be
     * addressed with Subsystem-level dynamics.
     */
    public void setVelocityUnlimited(
            double outputRad_S,
            double outputTorqueNm) {
        m_motor.setVelocity(
                outputRad_S * m_gearRatio,
                outputTorqueNm / m_gearRatio);
    }

    /**
     * Should actuate immediately. Enforces position limit using the encoder.
     * 
     * Previously this included an accel term. Acceleration should be
     * addressed with Subsystem-level dynamics.
     */
    public void setVelocity(
            double velocityRad_S,
            double torqueNm) {
        double posRad = getWrappedPositionRad();
        if (velocityRad_S < 0 && posRad < m_minPositionRad) {
            m_motor.stop();
            return;
        }
        if (velocityRad_S > 0 && posRad > m_maxPositionRad) {
            m_motor.stop();
            return;
        }
        m_motor.setVelocity(
                velocityRad_S * m_gearRatio,
                torqueNm / m_gearRatio);
    }

    /**
     * Choose a nearby unwrapped position.
     * 
     * Does not implement "spotting". If the nearest unwrapped
     * position is outside the bounds, it does nothing. If you
     * want spotting, use an AngularPositionServo.
     */
    public void setWrappedPosition(
            double positionRad,
            double velocityRad_S,
            double torqueNm) {
        double unwrappedMeasurement = getUnwrappedPositionRad();
        double dx = MathUtil.angleModulus(positionRad - unwrappedMeasurement);
        double unwrappedRad = unwrappedMeasurement + dx;
        if (unwrappedRad > getMaxPositionRad()) {
            return;
        }
        if (unwrappedRad < getMinPositionRad()) {
            return;
        }
        setUnwrappedPosition(unwrappedRad, velocityRad_S, torqueNm);
    }

    /**
     * Apply limits and gear ratio, and set the resulting motor position.
     * 
     * This is the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi.
     * 
     * Should actuate immediately.
     * 
     * Previously this included an accel term. Acceleration should be
     * addressed with Subsystem-level dynamics.
     */
    public void setUnwrappedPosition(
            double positionRad,
            double velocityRad_S,
            double torqueNm) {
        m_log_desired_position.log(() -> positionRad);
        if (positionRad < m_minPositionRad) {
            System.out.printf("WARNING: requested position %8.3f less than min %8.3f\n",
                    positionRad, m_minPositionRad);
            m_motor.stop();
            return;
        }
        if (positionRad > m_maxPositionRad) {
            System.out.printf("WARNING: requested position %8.3f more than max %8.3f\n",
                    positionRad, m_maxPositionRad);
            m_motor.stop();
            return;
        }
        m_motor.setUnwrappedPosition(
                positionRad * m_gearRatio,
                velocityRad_S * m_gearRatio,
                torqueNm / m_gearRatio);
    }

    public ModelR1 getUnwrappedMeasurement() {
        return new ModelR1(getUnwrappedPositionRad(), getVelocityRad_S());
    }

    /**
     * Value is updated in Robot.robotPeriodic().
     * 
     * @return velocity in rad/s
     */
    public double getVelocityRad_S() {
        return m_sensor.getVelocityRad_S();
    }

    /**
     * Returns the "wrapped" angular position, i.e. this dimension is cyclical, with
     * values beyond +/- pi mapped back to the +/- pi interval: 2pi is mapped to 0,
     * 5pi/4 is mapped to pi/4, etc.
     * 
     * @return the absolute 1:1 position of the mechanism in rad [-pi, pi]
     */
    public double getWrappedPositionRad() {
        return m_sensor.getWrappedPositionRad();
    }

    /** Unwrapped domain is infinite. */
    public double getUnwrappedPositionRad() {
        return m_sensor.getUnwrappedPositionRad();
    }

    /** Minimum unwrapped position. */
    public double getMinPositionRad() {
        return m_minPositionRad;
    }

    /** Maximum unwrapped position. */
    public double getMaxPositionRad() {
        return m_maxPositionRad;
    }

    public void stop() {
        m_motor.stop();
    }

    public void close() {
        m_motor.close();
    }

    public void periodic() {
        /**
         * The time constant is the minimum frequency to filter out,
         * and the period is the rate of sampling. Here, both are 50Hz.
         */
        // LinearFilter lowPassFilter = LinearFilter.movingAverage(5);
        m_motor.periodic();
        m_sensor.periodic();
        m_log_position.log(() -> getWrappedPositionRad());
        final double velocity = getVelocityRad_S();
        m_log_velocity.log(() -> velocity);
        double filt = lowPassFilter.calculate(velocity);
        m_log_filtered_velocity.log(() -> filt);
        double accel = (filt - m_velocity) / TimedRobot100.LOOP_PERIOD_S;
        m_velocity = filt;
        m_log_accel.log(() -> accel);

    }

    @Override
    public void play(double freq) {
        m_motor.play(freq);
    }

}
