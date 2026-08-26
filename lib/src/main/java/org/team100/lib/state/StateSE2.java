package org.team100.lib.state;

import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Describes the state of rigid body transformations in two dimensions, the
 * SE(2) manifold (x,y,theta), where each dimension is represented by position
 * and velocity.
 * 
 * This could be used for navigation, or for other applications of rigid-body
 * transforms in 2d, e.g. planar mechanisms.
 * 
 * This type is used for measurement and estimation, which is why it doesn't
 * include acceleration.
 * 
 * Note: the metric used here is not the SE(2) geodesic, it treats the XY plane
 * and rotation dimensions independently.
 */
public class StateSE2 {
    private final StateR1 m_x;
    private final StateR1 m_y;
    private final StateR1 m_theta;

    public StateSE2(StateR1 x, StateR1 y, StateR1 theta) {
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

    public StateSE2(Pose2d x, VelocitySE2 v) {
        this(
                new StateR1(x.getX(), v.x()),
                new StateR1(x.getY(), v.y()),
                new StateR1(x.getRotation().getRadians(), v.theta()));
    }

    /** Motionless with the specified pose */
    public StateSE2(Pose2d x) {
        this(x, VelocitySE2.ZERO);
    }

    /** Motionless at the origin with the specified heading */
    public StateSE2(Rotation2d x) {
        this(new Pose2d(0, 0, x));
    }

    /** Motionless at the origin */
    public StateSE2() {
        this(new StateR1(), new StateR1(), new StateR1());
    }

    public ControlSE2 control() {
        return new ControlSE2(m_x.control(), m_y.control(), m_theta.control());
    }

    public StateSE2 withTheta(double theta) {
        return new StateSE2(m_x, m_y, new StateR1(theta, m_theta.v()));
    }

    /** Component-wise difference (not geodesic) */
    public StateSE2 minus(StateSE2 other) {
        return new StateSE2(x().minus(other.x()), y().minus(other.y()), theta().minus(other.theta()));
    }

    /** Component-wise sum (not geodesic) */
    public StateSE2 plus(StateSE2 other) {
        return new StateSE2(x().plus(other.x()), y().plus(other.y()), theta().plus(other.theta()));
    }

    /**
     * Use the current velocity to evolve the position of each dimension
     * independently.
     * 
     * This does not describe geodesic paths in SE(2). For that, see Twist2d.
     */
    public StateSE2 evolve(double dt) {
        return new StateSE2(m_x.evolve(dt), m_y.evolve(dt), m_theta.evolve(dt));
    }

    /** All dimensions position and velocity are within (the same) tolerance */
    public boolean near(StateSE2 other, double tolerance) {
        return x().near(other.x(), tolerance)
                && y().near(other.y(), tolerance)
                && theta().near(other.theta(), tolerance);
    }

    public Pose2d pose() {
        return new Pose2d(m_x.x(), m_y.x(), new Rotation2d(m_theta.x()));
    }

    /** Translation of the pose. */
    public Translation2d translation() {
        return new Translation2d(m_x.x(), m_y.x());
    }

    public Rotation2d rotation() {
        return new Rotation2d(m_theta.x());
    }

    public VelocitySE2 velocity() {
        return new VelocitySE2(m_x.v(), m_y.v(), m_theta.v());
    }

    public VelocityR2 velocityR2() {
        return new VelocityR2(m_x.v(), m_y.v());
    }

    /** Robot-relative speeds. */
    public ChassisVelocities ChassisVelocities() {
        return SwerveKinodynamics.toInstantaneousChassisVelocities(velocity(), rotation());
    }

    public StateR1 x() {
        return m_x;
    }

    public StateR1 y() {
        return m_y;
    }

    public StateR1 theta() {
        return m_theta;
    }

    /** Point and pathwise velocity => StateSE2 */
    public static StateSE2 fromMovingPathPointSE2(PathSE2Point point, double velocityM_s) {
        WaypointSE2 pose = point.waypoint();
        Translation2d translation = pose.pose().getTranslation();
        double xx = translation.getX();
        double yx = translation.getY();
        double thetax = pose.pose().getRotation().getRadians();
        Rotation2d course = point.waypoint().course().toRotation();
        double xv = course.getCos() * velocityM_s;
        double yv = course.getSin() * velocityM_s;
        double thetav = point.waypoint().course().headingRate() * velocityM_s;
        return new StateSE2(
                new StateR1(xx, xv),
                new StateR1(yx, yv),
                new StateR1(thetax, thetav));
    }

    public String toString() {
        return "StateSE2(" + m_x + ", " + m_y + ", " + m_theta + ")";
    }
}