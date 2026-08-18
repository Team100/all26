package org.team100.lib.subsystems.swerve.kinodynamics.limiter;

import java.util.function.DoubleSupplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Maximum speed scales linearly with applied voltage. We also want to avoid
 * driving the battery voltage below the brown-out limit, so there's an extra
 * penalty for very low voltage.
 * 
 * Note the motors actually get a bit less voltage than the battery, due to
 * wiring resistance, but it's ok to neglect that effect.
 */
public class BatterySagSpeedLimit {
    private static final boolean DEBUG = false;

    private final DoubleLogger m_log_scale;
    private final SwerveKinodynamics m_dynamics;
    private final DoubleSupplier m_voltage;
    private final InterpolatingDoubleTreeMap m_table;

    public BatterySagSpeedLimit(
            LoggerFactory parent,
            SwerveKinodynamics dynamics,
            DoubleSupplier voltage) {
        LoggerFactory log = parent.type(this);
        m_log_scale = log.doubleLogger(Level.DEBUG, "scale");
        m_dynamics = dynamics;
        // there's a supplier here so that the tests don't need to use the
        // RobotController HAL, which sometimes mysteriously fails.
        m_voltage = voltage;
        m_table = new InterpolatingDoubleTreeMap();
        // 12v is spec voltage
        m_table.put(12.0, 1.0);
        // proportional down to 7v
        m_table.put(7.0, 7.0 / 12.0);
        // zero speed below 6v
        m_table.put(6.0, 0.0);
    }

    public double getMaxDriveVelocityM_S() {
        double scale = getScale();
        if (DEBUG) {
            System.out.printf("BatterySagSpeedLimit velocity scale %.5f\n", scale);
        }
        return scale * m_dynamics.getMaxDriveVelocityM_S();
    }

    public double getMaxAngleSpeedRad_S() {
        double scale = getScale();
        if (DEBUG) {
            System.out.printf("BatterySagSpeedLimit  omega scale %.5f\n", scale);
        }
        return scale * m_dynamics.getMaxAngleSpeedRad_S();
    }

    private double getScale() {
        double scale = m_table.get(m_voltage.getAsDouble());
        m_log_scale.log(() -> scale);
        return scale;
    }
}
