package org.team100.lib.subsystems.swerve.kinodynamics.limiter;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.AccelerationSE2Logger;
import org.team100.lib.logging.LoggerFactory.VelocitySE2Logger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

/**
 * Limits acceleration to avoid tipping over.
 */
public class FieldRelativeCapsizeLimiter {
    private static final boolean DEBUG = false;
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;

    private final DoubleLogger m_log_scale;
    private final AccelerationSE2Logger m_log_accel;
    private final VelocitySE2Logger m_log_prev;
    private final VelocitySE2Logger m_log_target;

    private final SwerveKinodynamics limits;

    public FieldRelativeCapsizeLimiter(
            LoggerFactory parent,
            SwerveKinodynamics m_limits) {
        LoggerFactory log = parent.type(this);
        m_log_scale = log.doubleLogger(Level.DEBUG, "scale");
        m_log_accel = log.AccelerationSE2Logger(Level.TRACE, "accel");
        m_log_prev = log.VelocitySE2Logger(Level.TRACE, "prev");
        m_log_target = log.VelocitySE2Logger(Level.TRACE, "target");
        limits = m_limits;
    }

    public VelocitySE2 apply(
            VelocitySE2 prev,
            VelocitySE2 target) {
        m_log_prev.log(() -> prev);
        m_log_target.log(() -> target);
        // Acceleration required to achieve the target.
        AccelerationSE2 accel = target.accel(prev, DT);
        m_log_accel.log(() -> accel);
        double a = accel.norm();
        if (a < 1e-6) {
            // Zero acceleration.
            a = 0;
        }
        double scale = scale(a);
        m_log_scale.log(() -> scale);
        // v1 = v0 + a dt
        VelocitySE2 result = accel.times(scale).evolve(prev, DT);
        if (DEBUG) {
            System.out.printf("FieldRelativeCapsizeLimiter prev %s target %s accel %s scale %5.2f result %s\n",
                    prev, target, accel, scale, result);
        }
        return result;
    }

    double scale(double a) {
        return Math.min(1, limits.getMaxCapsizeAccelM_S2() / a);
    }
}
