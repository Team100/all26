package org.team100.lib.mechanism;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.Motor;
import org.team100.lib.music.Player;
import org.team100.lib.sensor.position.incremental.IncrementalEncoder;

/**
 * Uses a motor, gears, and a wheel to produce linear output, e.g. a drive wheel
 * or conveyor belt.
 * 
 * The limits used to be enforced by a proxy, but now they're here: it seems
 * simpler that way.
 * 
 * Computes a noisy estimate of actual acceleration using backwards finite
 * difference.
 */
public class LinearMechanism implements Player {
    private static final boolean DEBUG = false;

    private final Motor m_motor;
    private final IncrementalEncoder m_encoder;
    private final double m_gearRatio;
    private final double m_wheelRadiusM;
    private final double m_minPositionM;
    private final double m_maxPositionM;
    private final DoubleLogger m_log_accel;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_position;

    public LinearMechanism(
            LoggerFactory parent,
            Motor motor,
            IncrementalEncoder encoder,
            double gearRatio,
            double wheelDiameterM,
            double minPositionM,
            double maxPositionM) {
        m_motor = motor;
        m_encoder = encoder;
        m_gearRatio = gearRatio;
        m_wheelRadiusM = wheelDiameterM / 2;
        m_minPositionM = minPositionM;
        m_maxPositionM = maxPositionM;
        LoggerFactory log = parent.type(this);
        m_log_accel = log.doubleLogger(Level.DEBUG, "accel (m_s2)");
        m_log_velocity = log.doubleLogger(Level.DEBUG, "velocity (m_s)");
        m_log_position = log.doubleLogger(Level.DEBUG, "position (m)");
    }

    /** Should actuate immediately. Use for homing. */
    public void setDutyCycleUnlimited(double output) {
        m_motor.setDutyCycle(output);
    }

    /** Should actuate immediately. Limits position using the encoder. */
    public void setDutyCycle(double output) {
        double posM = getPositionM();
        if (output < 0 && posM < m_minPositionM) {
            m_motor.stop();
            return;
        }
        if (output > 0 && posM > m_maxPositionM) {
            m_motor.stop();
            return;
        }
        m_motor.setDutyCycle(output);
    }

    /** For tuning friction. */
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    public void setForceLimit(double forceN) {
        m_motor.setTorqueLimit(forceN * m_wheelRadiusM / m_gearRatio);
    }

    /**
     * Should actuate immediately. Use for homing.
     *
     * Previously this included an accel term. Acceleration should be
     * addressed with Subsystem-level dynamics.
     */
    public void setVelocityUnlimited(double outputVelocityM_S, double outputForceN) {
        m_motor.setVelocity(
                (outputVelocityM_S / m_wheelRadiusM) * m_gearRatio,
                outputForceN * m_wheelRadiusM / m_gearRatio);
    }

    /**
     * Should actuate immediately. Limits position using the encoder.
     * 
     * Previously this included an accel term. Acceleration should be
     * addressed with Subsystem-level dynamics.
     */
    public void setVelocity(
            double outputVelocityM_S,
            double outputForceN) {
        if (DEBUG) {
            System.out.printf("velocity %6.3f\n", outputVelocityM_S);
        }
        double posM = getPositionM();
        if (outputVelocityM_S < 0 && posM < m_minPositionM) {
            m_motor.stop();
            return;
        }
        if (outputVelocityM_S > 0 && posM > m_maxPositionM) {
            m_motor.stop();
            return;
        }
        m_motor.setVelocity(
                (outputVelocityM_S / m_wheelRadiusM) * m_gearRatio,
                outputForceN * m_wheelRadiusM / m_gearRatio);
    }

    /**
     * Apply limits, use wheel diameter, and gear ratio, and set the resulting
     * motor position.
     * 
     * Should actuate immediately.
     * 
     * Previously this included an accel term. Acceleration should be
     * addressed with Subsystem-level dynamics.
     */
    public void setPosition(
            double positionM,
            double velocityM_S,
            double forceN) {
        if (positionM < m_minPositionM) {
            m_motor.stop();
            return;
        }
        if (positionM > m_maxPositionM) {
            m_motor.stop();
            return;
        }
        m_motor.setUnwrappedPosition(
                (positionM / m_wheelRadiusM) * m_gearRatio,
                (velocityM_S / m_wheelRadiusM) * m_gearRatio,
                forceN * m_wheelRadiusM / m_gearRatio);
    }

    /** Nearly cached. */
    public double getPositionM() {
        double positionRad = m_encoder.getUnwrappedPositionRad();
        return positionRad * m_wheelRadiusM / m_gearRatio;
    }

    /** Nearly cached. */
    public double getVelocityM_S() {
        double velocityRad_S = m_encoder.getVelocityRad_S();
        return velocityRad_S * m_wheelRadiusM / m_gearRatio;
    }

    /** Nearly cached. */
    public double getAccelerationM_S2() {
        double accelRad_S2 = m_encoder.getAccelerationRad_S2();
        return accelRad_S2 * m_wheelRadiusM / m_gearRatio;
    }

    /** This is not "hold position" this is "torque off". */
    public void stop() {
        m_motor.stop();
    }

    public void close() {
        m_motor.close();
        m_encoder.close();
    }

    /** For logging. */
    public void periodic() {
        m_motor.periodic();
        m_encoder.periodic();
        m_log_position.log(this::getPositionM);
        m_log_velocity.log(this::getVelocityM_S);
        m_log_accel.log(this::getAccelerationM_S2);
    }

    @Override
    public void play(double freq) {
        m_motor.play(freq);
    }

}
