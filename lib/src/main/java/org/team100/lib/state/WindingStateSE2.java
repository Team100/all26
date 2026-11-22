package org.team100.lib.state;

import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.team100.lib.geometry.se2_universal_cover.WindingWaypointSE2;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.Interpolatable;
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
public class WindingStateSE2 implements Interpolatable<WindingStateSE2> {
    private final StateR1 m_x;
    private final StateR1 m_y;
    private final StateR1 m_theta;

    public WindingStateSE2(StateR1 x, StateR1 y, StateR1 theta) {
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

    public WindingStateSE2(WindingPose2d x, VelocitySE2 v) {
        this(
                new StateR1(x.getX(), v.x()),
                new StateR1(x.getY(), v.y()),
                new StateR1(x.getRotation(), v.theta()));
    }

    /** Motionless with the specified pose */
    public WindingStateSE2(WindingPose2d x) {
        this(x, VelocitySE2.ZERO);
    }

    public WindingStateSE2(StateSE2 s) {
        this(s.x(), s.y(), s.theta());
    }

    /** Motionless at the origin */
    public WindingStateSE2() {
        this(new StateR1(), new StateR1(), new StateR1());
    }

    public StateSE2 stateSE2() {
        return new StateSE2(m_x, m_y, m_theta);
    }

    public WindingControlSE2 control() {
        return new WindingControlSE2(m_x.control(), m_y.control(), m_theta.control());
    }

    public WindingStateSE2 withTheta(double theta) {
        return new WindingStateSE2(m_x, m_y, new StateR1(theta, m_theta.v()));
    }

    /** Component-wise difference (not geodesic) */
    public WindingStateSE2 minus(WindingStateSE2 other) {
        return new WindingStateSE2(x().minus(other.x()), y().minus(other.y()), theta().minus(other.theta()));
    }

    /** Component-wise sum (not geodesic) */
    public WindingStateSE2 plus(WindingStateSE2 other) {
        return new WindingStateSE2(x().plus(other.x()), y().plus(other.y()), theta().plus(other.theta()));
    }

    /**
     * Use the current velocity to evolve the position of each dimension
     * independently.
     * 
     * This does not describe geodesic paths in SE(2). For that, see Twist2d.
     */
    public WindingStateSE2 evolve(double dt) {
        return new WindingStateSE2(m_x.evolve(dt), m_y.evolve(dt), m_theta.evolve(dt));
    }

    /** All dimensions position and velocity are within (the same) tolerance */
    public boolean near(WindingStateSE2 other, double tolerance) {
        return x().near(other.x(), tolerance)
                && y().near(other.y(), tolerance)
                && theta().near(other.theta(), tolerance);
    }

    public WindingPose2d pose() {
        return new WindingPose2d(m_x.x(), m_y.x(), m_theta.x());
    }

    /** Translation of the pose. */
    public Translation2d translation() {
        return new Translation2d(m_x.x(), m_y.x());
    }

    public double rotation() {
        return m_theta.x();
    }

    public VelocitySE2 velocity() {
        return new VelocitySE2(m_x.v(), m_y.v(), m_theta.v());
    }

    public VelocityR2 velocityR2() {
        return new VelocityR2(m_x.v(), m_y.v());
    }

    /** Robot-relative speeds. */
    public ChassisVelocities ChassisVelocities() {
        return SwerveKinodynamics.toInstantaneousChassisVelocities(velocity(), new Rotation2d(rotation()));
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
    public static WindingStateSE2 fromMovingPathPointSE2(PathSE2Point point, double velocityM_s) {
        WindingWaypointSE2 pose = point.waypoint();
        Translation2d translation = pose.pose().getTranslation();
        double xx = translation.getX();
        double yx = translation.getY();
        double thetax = pose.pose().getRotation();
        Rotation2d course = point.waypoint().course().toRotation();
        double xv = course.getCos() * velocityM_s;
        double yv = course.getSin() * velocityM_s;
        double thetav = point.waypoint().course().headingRate() * velocityM_s;
        return new WindingStateSE2(
                new StateR1(xx, xv),
                new StateR1(yx, yv),
                new StateR1(thetax, thetav));
    }

    @Override
    public String toString() {
        return "StateSE2(" + m_x + ", " + m_y + ", " + m_theta + ")";
    }

    @Override
    public WindingStateSE2 interpolate(WindingStateSE2 end, double t) {
        WindingPose2d poseLerp = pose().interpolate(end.pose(), t);
        VelocitySE2 velocityLerp = velocity().interpolate(end.velocity(), t);
        return new WindingStateSE2(poseLerp, velocityLerp);
    }
}