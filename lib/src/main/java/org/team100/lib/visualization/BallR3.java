package org.team100.lib.visualization;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.r3.GlobalVelocityR3;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.targeting.Drag;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.system.NumericalIntegration;

/**
 * Simulated projectile in three dimensions (x, y, z)
 */
public class BallR3 implements Ball {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;
    private final DoubleLogger m_log_ball_altitude;
    private final DoubleArrayLogger m_log_field_ball;
    private final Drag m_drag;
    private final Supplier<ModelSE2> m_robot;
    private final Supplier<Rotation2d> m_azimuth;
    private final Supplier<Rotation2d> m_elevation;
    private final DoubleSupplier m_speed;
    private final double m_omega;

    // null when contained in robot.
    // robot location at launch
    Translation3d m_location;
    // current state for drag model
    Matrix<N6, N1> m_x;
    // azimuth at launch
    Rotation2d m_az;

    /**
     * @param field     log
     * @param drag      drag model
     * @param robot     state (pose2d, velocityR3)
     * @param azimuth   absolute
     * @param elevation absolute
     * @param speed     muzzle speed
     * @param omega     spin
     */
    public BallR3(
            LoggerFactory parent,
            LoggerFactory field,
            Drag drag,
            Supplier<ModelSE2> robot,
            Supplier<Rotation2d> azimuth,
            Supplier<Rotation2d> elevation,
            DoubleSupplier speed,
            double omega) {
        LoggerFactory log = parent.type(this);
        m_log_ball_altitude = log.doubleLogger(Level.COMP, "altitude");
        m_log_field_ball = field.doubleArrayLogger(Level.COMP, "ball");
        m_drag = drag;
        m_robot = robot;
        m_azimuth = azimuth;
        m_elevation = elevation;
        m_speed = speed;
        m_omega = omega;
    }

    @Override
    public void launch() {
        // Velocity due only to the gun
        GlobalVelocityR3 v = GlobalVelocityR3.fromPolar(
                m_azimuth.get(), m_elevation.get(), m_speed.getAsDouble());
        // velocity due to robot translation
        GlobalVelocityR3 mv = GlobalVelocityR3.fromSe2(m_robot.get().velocity());
        // Initial position is on the floor.
        m_location = new Translation3d(m_robot.get().pose().getTranslation());
        GlobalVelocityR3 m_velocity = v.plus(mv);
        // velocity in the XY plane
        double vxy = m_velocity.normXY();
        double vz = m_velocity.z();
        m_x = VecBuilder.fill(0, 0, 0, vxy, vz, m_omega);
        m_az = m_azimuth.get();
    }

    @Override
    public void fly() {
        Matrix<N6, N1> x = NumericalIntegration.rk4(m_drag, m_x, DT);
        if (x.get(1, 0) >= 0) {
            // only update if above the floor
            m_x = x;
        }
    }

    @Override
    public void reset() {
        m_location = null;
        m_x = null;
    }

    @Override
    public void periodic() {
        m_log_ball_altitude.log(this::altitude);
        m_log_field_ball.log(this::poseArray);
    }

    private double[] poseArray() {
        Translation3d t = location();
        return new double[] { t.getX(), t.getY(), 0 };
    }

    private double altitude() {
        Translation3d t = location();
        return t.getZ();
    }

    Translation3d location() {
        if (m_x == null) {
            // The ball is riding with the robot.
            return new Translation3d(m_robot.get().translation());
        }
        double xy = m_x.get(0, 0);
        double z = m_x.get(1, 0);
        Translation3d relative = new Translation3d(
                xy * m_az.getCos(),
                xy * m_az.getSin(),
                z);
        return m_location.plus(relative);
    }

    // for testing
    GlobalVelocityR3 velocity() {
        if (m_x == null) {
            return GlobalVelocityR3.fromSe2(m_robot.get().velocity());
        }
        double vxy = m_x.get(3, 0);
        double vz = m_x.get(4, 0);
        return new GlobalVelocityR3(
                vxy * m_az.getCos(),
                vxy * m_az.getSin(),
                vz);
    }
}
