package org.team100.lib.state;

import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
 * 
 * TODO: rename this to StateSE2
 */
public class ModelSE2 {
    private final ModelR1 m_x;
    private final ModelR1 m_y;
    private final ModelR1 m_theta;

    public ModelSE2(ModelR1 x, ModelR1 y, ModelR1 theta) {
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

    public ModelSE2(Pose2d x, VelocitySE2 v) {
        this(
                new ModelR1(x.getX(), v.x()),
                new ModelR1(x.getY(), v.y()),
                new ModelR1(x.getRotation().getRadians(), v.theta()));
    }

    /** Motionless with the specified pose */
    public ModelSE2(Pose2d x) {
        this(x, VelocitySE2.ZERO);
    }

    /** Motionless at the origin with the specified heading */
    public ModelSE2(Rotation2d x) {
        this(new Pose2d(0, 0, x));
    }

    /** Motionless at the origin */
    public ModelSE2() {
        this(new ModelR1(), new ModelR1(), new ModelR1());
    }

    public ControlSE2 control() {
        return new ControlSE2(m_x.control(), m_y.control(), m_theta.control());
    }

    public ModelSE2 withTheta(double theta) {
        return new ModelSE2(m_x, m_y, new ModelR1(theta, m_theta.v()));
    }

    /** Component-wise difference (not geodesic) */
    public ModelSE2 minus(ModelSE2 other) {
        return new ModelSE2(x().minus(other.x()), y().minus(other.y()), theta().minus(other.theta()));
    }

    /** Component-wise sum (not geodesic) */
    public ModelSE2 plus(ModelSE2 other) {
        return new ModelSE2(x().plus(other.x()), y().plus(other.y()), theta().plus(other.theta()));
    }

    /**
     * Use the current velocity to evolve the position of each dimension
     * independently.
     * 
     * This does not describe geodesic paths in SE(2). For that, see Twist2d.
     */
    public ModelSE2 evolve(double dt) {
        return new ModelSE2(m_x.evolve(dt), m_y.evolve(dt), m_theta.evolve(dt));
    }

    /** All dimensions position and velocity are within (the same) tolerance */
    public boolean near(ModelSE2 other, double tolerance) {
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
    public ChassisSpeeds chassisSpeeds() {
        return SwerveKinodynamics.toInstantaneousChassisSpeeds(velocity(), rotation());
    }

    public ModelR1 x() {
        return m_x;
    }

    public ModelR1 y() {
        return m_y;
    }

    public ModelR1 theta() {
        return m_theta;
    }

    /** Point and pathwise velocity => ModelSE2 */
    public static ModelSE2 fromMovingPathPointSE2(PathSE2Point point, double velocityM_s) {
        WaypointSE2 pose = point.waypoint();
        Translation2d translation = pose.pose().getTranslation();
        double xx = translation.getX();
        double yx = translation.getY();
        double thetax = pose.pose().getRotation().getRadians();
        Rotation2d course = point.waypoint().course().toRotation();
        double xv = course.getCos() * velocityM_s;
        double yv = course.getSin() * velocityM_s;
        double thetav = point.waypoint().course().headingRate() * velocityM_s;
        return new ModelSE2(
                new ModelR1(xx, xv),
                new ModelR1(yx, yv),
                new ModelR1(thetax, thetav));
    }

    public String toString() {
        return "ModelSE2(" + m_x + ", " + m_y + ", " + m_theta + ")";
    }
}