package org.team100.lib.subsystems.swerve.kinodynamics.limiter;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.AccelerationSE2Logger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

/**
 * Limits cartesian and rotational acceleration.
 * 
 * The limit is applied to the entire holonomic acceleration together, not to
 * the individual components, so if you exceed the rotational limit, both
 * rotation *and* translational speed will be reduced.
 */
public class FieldRelativeAccelerationLimiter {
    private static final boolean DEBUG = false;
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;

    private final DoubleLogger m_log_scale;
    private final AccelerationSE2Logger m_log_accel;
    private final SwerveKinodynamics m_limits;
    private final double m_cartesianScale;
    private final double m_alphaScale;

    /**
     * @param parent
     * @param limits         Absolute maxima.
     * @param cartesianScale Fraction of cartesian maximum to allow.
     * @param alphaScale     Fraction of rotational maximum to allow.
     */
    public FieldRelativeAccelerationLimiter(
            LoggerFactory parent,
            SwerveKinodynamics limits,
            double cartesianScale,
            double alphaScale) {
        LoggerFactory log = parent.type(this);
        m_log_scale = log.doubleLogger(Level.DEBUG, "scale");
        m_log_accel = log.AccelerationSE2Logger(Level.TRACE, "accel");
        m_limits = limits;
        m_cartesianScale = cartesianScale;
        m_alphaScale = alphaScale;
    }

    public VelocitySE2 apply(
            VelocitySE2 prev,
            VelocitySE2 target) {
        // Acceleration required to achieve the target.
        AccelerationSE2 accel = target.accel(prev, DT);
        m_log_accel.log(() -> accel);
        double cartesianScale = cartesianScale(prev, target, accel);
        double alphaScale = alphaScale(accel);
        double scale = Math.min(cartesianScale, alphaScale);
        m_log_scale.log(() -> scale);
        // v1 = v0 + a dt
        VelocitySE2 result = accel.times(scale).evolve(prev, DT);
        if (DEBUG) {
            System.out.printf(
                    "FieldRelativeAccelerationLimiter prev %s target %s accel %s cartesian scale %5.2f alpha scale %5.2f total scale %5.2f result %s\n",
                    prev, target, accel, cartesianScale, alphaScale, scale, result);
        }
        return result;
    }

    double cartesianScale(
            VelocitySE2 prev,
            VelocitySE2 target,
            AccelerationSE2 accel) {
        double a = accel.norm();
        if (Math.abs(a) < 1e-6) {
            // Avoid divide-by-zero.
            return 1.0;
        }
        double accelLimit = SwerveUtil.getAccelLimit(m_limits, 1, m_cartesianScale, prev, target);
        // At full speed, both a and accelLimit are around zero.
        if (a < accelLimit) {
            a = accelLimit;
        }
        return Math.min(1, accelLimit / a);
    }

    private double alphaScale(AccelerationSE2 accel) {
        double a = accel.theta();
        if (Math.abs(a) < 1e-6) {
            // Avoid divide-by-zero.
            return 1.0;
        }
        double accelLimit = m_alphaScale * m_limits.getMaxAngleAccelRad_S2();
        if (a < accelLimit) {
            a = accelLimit;
        }
        return Math.min(1, accelLimit / a);
    }

}
