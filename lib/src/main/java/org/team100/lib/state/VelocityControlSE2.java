package org.team100.lib.state;

import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.hid.Velocity;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;

/**
 * For velocity control in SE2, where position is not
 * directly controlled, e.g. the swerve drive.
 */
public class VelocityControlSE2 {
    public static final VelocityControlSE2 ZERO = new VelocityControlSE2(0, 0, 0);

    private final VelocityControlR1 m_x;
    private final VelocityControlR1 m_y;
    private final VelocityControlR1 m_theta;

    public VelocityControlSE2(VelocityControlR1 x, VelocityControlR1 y, VelocityControlR1 theta) {
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

    /** Velocity only. */
    public VelocityControlSE2(double x, double y, double theta) {
        this(new VelocityControlR1(x, 0),
                new VelocityControlR1(y, 0),
                new VelocityControlR1(theta, 0));
    }

    /** Velocity only. */
    public VelocityControlSE2(VelocitySE2 v) {
        this(v.x(), v.y(), v.theta());
    }

    public VelocityControlSE2(VelocitySE2 v, AccelerationSE2 a) {
        this(new VelocityControlR1(v.x(), a.x()),
                new VelocityControlR1(v.y(), a.y()),
                new VelocityControlR1(v.theta(), a.theta()));
    }

    public VelocityControlR1 x() {
        return m_x;
    }

    public VelocityControlR1 y() {
        return m_y;
    }

    public VelocityControlR1 theta() {
        return m_theta;
    }

    public VelocitySE2 velocity() {
        return new VelocitySE2(m_x.v(), m_y.v(), m_theta.v());
    }

    public AccelerationSE2 acceleration() {
        return new AccelerationSE2(m_x.a(), m_y.a(), m_theta.a());
    }

    public VelocityControlSE2 plus(VelocityControlSE2 other) {
        return new VelocityControlSE2(
                m_x.plus(other.x()),
                m_y.plus(other.y()),
                m_theta.plus(other.theta()));
    }

    /**
     * Integrate the velocity from the initial pose for time dt.
     * 
     * TODO: add acceleration term
     */
    public Pose2d integrate(Pose2d initial, double dt) {
        return new Pose2d(
                initial.getX() + m_x.v() * dt,
                initial.getY() + m_y.v() * dt,
                initial.getRotation().plus(new Rotation2d(m_theta.v() * dt)));
    }

    /** Velocity only. */
    public static VelocityControlSE2 fromVector(Vector<N3> v) {
        return new VelocityControlSE2(v.get(0), v.get(1), v.get(2));
    }

    /**
     * Scales driver input to field-relative velocity control.
     * 
     * This makes no attempt to address infeasibilty, it just multiplies.
     * 
     * TODO: add support for acceleration via backwards finite difference.
     * 
     * @param v        [-1,1]
     * @param maxSpeed meters per second
     * @param maxRot   radians per second
     * @return meters and rad per second as specified by speed limits
     */
    public static VelocityControlSE2 scale(Velocity v, double maxSpeed, double maxRot) {
        return new VelocityControlSE2(
                maxSpeed * MathUtil.clamp(v.x(), -1, 1),
                maxSpeed * MathUtil.clamp(v.y(), -1, 1),
                maxRot * MathUtil.clamp(v.theta(), -1, 1));
    }
}
