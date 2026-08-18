package org.team100.lib.trajectory.se2.constraint;

import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Point;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Limits torque due to acceleration along the path.
 * 
 * This is for the Calgames PRR mechanism, so the center of rotation is the
 * origin (middle of the robot on the floor, if it were calibrated; at
 * the moment, it's the shoulder axis when at rest.)
 * 
 * F = ma for torque is the cross product:
 * 
 * \tau = m(\vec{r} \times \vec{a})
 * 
 * We know the direction of \vec{a}, so use a scalar and a unit vector:
 * 
 * \tau = ma(\vec{r} \times \vec{u})
 * 
 * So
 * 
 * a = \tau / (m (\vec{r} \times \vec{u}))
 * 
 * For capsize torque, mass is roughly 50 kg, and the lever from COM to wheel is
 * about 0.3 m, so about 150 Nm. Experimentally, we find that we want much lower
 * limits than this, to avoid coming anywhere near the limit.
 * 
 * See https://en.wikipedia.org/wiki/Angular_acceleration
 */
public class TorqueConstraint implements TimingConstraint {
    private static final boolean DEBUG = false;
    /** Approximate mass of end effector in kg. */
    private static final double M = 6.0;
    /** Maximum allowed torque in Nm. */
    private final double m_maxTorque;

    /**
     * @param maxTorque max torque in Nm
     */
    public TorqueConstraint(double maxTorque) {
        m_maxTorque = maxTorque;
    }

    @Override
    public double maxV(PathSE2Point point) {
        // Do not constrain velocity.
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public double maxAccel(PathSE2Point point, double velocityM_S) {
        return getA(point);
    }

    @Override
    public double maxDecel(PathSE2Point point, double velocity) {
        return -getA(point);
    }

    private double getA(PathSE2Point state) {
        WaypointSE2 pose = state.waypoint();
        Rotation2d course = pose.course().toRotation();
        // acceleration unit vector
        Translation2d u = new Translation2d(1.0, course);
        Translation2d r = pose.pose().getTranslation();
        double cross = r.getX() * u.getY() - r.getY() * u.getX();
        double a = Math.abs(m_maxTorque / (M * cross));
        if (DEBUG) {
            System.out.printf("Torque Constraint a: %6.3f p: %s r: %6.3f course: %6.3f\n",
                    a, pose, r.getNorm(), course.getRadians());
        }
        return a;
    }
}
