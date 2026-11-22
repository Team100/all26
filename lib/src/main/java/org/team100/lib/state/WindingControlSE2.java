package org.team100.lib.state;

import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Describes the state of rigid body transformations in two dimensions, the
 * SE(2) manifold (x,y,theta), where each dimension is represented by position,
 * velocity, and acceleration.
 * 
 * This could be used for navigation, or for other applications of rigid-body
 * transforms in 2d, e.g. planar mechanisms.
 * 
 * This type is used for control, which is why it includes acceleration
 * 
 * Do not try to use zero as an initial location; always initialize with the
 * current location.
 * 
 * Be careful of the context: this specifies control relative to some
 * coordinate system, often the global (field) one, but not always.
 * 
 * Note: the metric used here is not the SE(2) geodesic, it treats the XY plane
 * and rotation dimensions independently.
 */
public class WindingControlSE2 {
    /** Meters */
    private final ControlR1 m_x;
    /** Meters */
    private final ControlR1 m_y;
    /** Radians */
    private final ControlR1 m_theta;

    public WindingControlSE2(ControlR1 x, ControlR1 y, ControlR1 theta) {
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

    public WindingControlSE2(WindingPose2d x, VelocitySE2 v) {
        this(
                new ControlR1(x.getX(), v.x(), 0),
                new ControlR1(x.getY(), v.y(), 0),
                new ControlR1(x.getRotation(), v.theta(), 0));
    }

    public WindingControlSE2(WindingPose2d x, VelocitySE2 v, AccelerationSE2 a) {
        this(
                new ControlR1(x.getX(), v.x(), a.x()),
                new ControlR1(x.getY(), v.y(), a.y()),
                new ControlR1(x.getRotation(), v.theta(), a.theta()));
    }

    public WindingControlSE2(WindingPose2d x) {
        this(x, VelocitySE2.ZERO);
    }

    public ControlSE2 controlSE2() {
        return new ControlSE2(m_x, m_y, m_theta);
    }

    public static WindingControlSE2 zero() {
        return new WindingControlSE2(new ControlR1(), new ControlR1(), new ControlR1());
    }

    public WindingStateSE2 state() {
        return new WindingStateSE2(m_x.state(), m_y.state(), m_theta.state());
    }

    /** Component-wise difference (not geodesic) */
    public WindingControlSE2 minus(WindingControlSE2 other) {
        return new WindingControlSE2(x().minus(other.x()), y().minus(other.y()), theta().minus(other.theta()));
    }

    /** Component-wise sum (not geodesic) */
    public WindingControlSE2 plus(WindingControlSE2 other) {
        return new WindingControlSE2(x().plus(other.x()), y().plus(other.y()), theta().plus(other.theta()));
    }

    public boolean near(WindingControlSE2 other, double tolerance) {
        return x().near(other.x(), tolerance)
                && y().near(other.y(), tolerance)
                && theta().near(other.theta(), tolerance);
    }

    public WindingPose2d pose() {
        return new WindingPose2d(m_x.x(), m_y.x(), rotation());
    }

    /** Translation of the pose */
    public Translation2d translation() {
        return new Translation2d(m_x.x(), m_y.x());
    }

    /** Can exceed pi */
    public double rotation() {
        return m_theta.x();
    }

    public VelocitySE2 velocity() {
        return new VelocitySE2(m_x.v(), m_y.v(), m_theta.v());
    }

    public VelocityControlSE2 velocityControl() {
        return new VelocityControlSE2(
                m_x.velocityControl(),
                m_y.velocityControl(),
                m_theta.velocityControl());
    }

    /** Robot-relative speeds */
    public ChassisVelocities ChassisVelocities() {
        return SwerveKinodynamics.toInstantaneousChassisVelocities(velocity(), new Rotation2d(rotation()));
    }

    public AccelerationSE2 acceleration() {
        return new AccelerationSE2(m_x.a(), m_y.a(), m_theta.a());
    }

    public ControlR1 x() {
        return m_x;
    }

    public ControlR1 y() {
        return m_y;
    }

    public ControlR1 theta() {
        return m_theta;
    }

    public String toString() {
        return "ControlSE2(" + m_x + ", " + m_y + ", " + m_theta + ")";
    }

}