package org.team100.lib.subsystems.swerve.kinodynamics.limiter;

import java.util.function.DoubleSupplier;

import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.VelocitySE2Logger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

/**
 * Makes drivetrain input feasible.
 * 
 * Keeps the current setpoint, to avoid round-tripping through the pose
 * estimator. Remember to update the setpoint!
 */
public class SwerveLimiter {
    private static final boolean DEBUG = false;

    private final DoubleLogger m_log_norm;
    private final DoubleLogger m_log_normIn;

    private final VelocitySE2Logger m_log_next;

    private final FieldRelativeVelocityLimiter m_velocityLimiter;
    private final FieldRelativeCapsizeLimiter m_capsizeLimiter;
    private final FieldRelativeAccelerationLimiter m_accelerationLimiter;
    // Velocity expected at the current time, i.e. the previous time step's desire.
    private VelocitySE2 m_current;

    public SwerveLimiter(LoggerFactory parent, SwerveKinodynamics dynamics, DoubleSupplier voltage) {
        LoggerFactory log = parent.type(this);
        m_log_norm = log.doubleLogger(Level.DEBUG, "norm");
        m_log_normIn = log.doubleLogger(Level.TRACE, "norm in");
        m_log_next = log.VelocitySE2Logger(Level.TRACE, "next");

        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(log, dynamics, voltage);
        m_velocityLimiter = new FieldRelativeVelocityLimiter(log, limit);
        m_capsizeLimiter = new FieldRelativeCapsizeLimiter(log, dynamics);

        // Use the absolute maximums.
        final double cartesianScale = 1.0;
        final double alphaScale = 1.0;
        m_accelerationLimiter = new FieldRelativeAccelerationLimiter(log, dynamics, cartesianScale, alphaScale);
    }

    /**
     * Find a feasible setpoint in the direction of the target, and remember it for
     * next time.
     */
    public VelocitySE2 apply(VelocitySE2 control) {
        VelocitySE2 nextReference = control;
        m_log_next.log(() -> nextReference);
        m_log_normIn.log(nextReference::norm);
        if (DEBUG) {
            System.out.printf("nextReference %s\n", nextReference);
        }
        if (m_current == null)
            m_current = nextReference;

        // First, limit the goal to a feasible velocity.
        VelocitySE2 result = m_velocityLimiter.apply(nextReference);
        if (DEBUG) {
            System.out.printf("velocity limited %s\n", result);
        }

        // then limit acceleration towards that goal to avoid capsize
        result = m_capsizeLimiter.apply(m_current, result);
        if (DEBUG) {
            System.out.printf("capsize limited %s\n", result);
        }

        // Finally, limit acceleration further, using motor physics.
        result = m_accelerationLimiter.apply(m_current, result);
        if (DEBUG) {
            System.out.printf("accel limited %s\n", result);
        }

        updateSetpoint(result);

        if (DEBUG) {
            System.out.printf("result %s\n", result);
        }
        m_log_norm.log(result::norm);

        return result;
    }

    /**
     * Set the current setpoint to the current velocity measurement.
     * This is required to make resumption of manual control smooth.
     */
    public void updateSetpoint(VelocitySE2 setpoint) {
        m_current = setpoint;
    }

}
