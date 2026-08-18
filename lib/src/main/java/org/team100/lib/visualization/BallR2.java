package org.team100.lib.visualization;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Simulated projectile in XY plane, uses constant velocity, continues forever.
 * 
 * Provides Field2d visualization using the name "ball".
 */
public class BallR2 implements Ball {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;
    private final DoubleArrayLogger m_log_field_ball;
    private final Supplier<ModelSE2> m_robot;
    private final Supplier<Rotation2d> m_azimuth;
    /** Projectile speed m/s */
    private final DoubleSupplier m_speed;

    // null when contained in robot.
    Translation2d m_location;
    VelocityR2 m_velocity;

    /**
     * @param field   log
     * @param robot   state (pose2d, velocitySE2)
     * @param azimuth absolute
     * @param speed   muzzle speed
     */
    public BallR2(
            LoggerFactory field,
            Supplier<ModelSE2> robot,
            Supplier<Rotation2d> azimuth,
            DoubleSupplier speed) {
        m_log_field_ball = field.doubleArrayLogger(Level.COMP, "ball");
        m_robot = robot;
        m_azimuth = azimuth;
        m_speed = speed;
    }

    @Override
    public void launch() {
        // Velocity due only to the gun
        VelocityR2 v = VelocityR2.fromPolar(m_azimuth.get(), m_speed.getAsDouble());
        // Velocity due to robot translation
        VelocityR2 mv = VelocityR2.fromSe2(m_robot.get().velocity());
        // Initial position is at the robot center.
        m_location = m_robot.get().pose().getTranslation();
        // Initial velocity.
        m_velocity = v.plus(mv);
    }

    @Override
    public void fly() {
        m_location = m_velocity.integrate(m_location, DT);
    }

    @Override
    public void reset() {
        m_location = null;
    }

    @Override
    public void periodic() {
        m_log_field_ball.log(this::poseArray);
    }

    private double[] poseArray() {
        Translation2d t = location();
        return new double[] { t.getX(), t.getY(), 0 };
    }

    private Translation2d location() {
        if (m_location == null) {
            // The ball is riding with the robot.
            return m_robot.get().translation();
        }
        return m_location;
    }
}
