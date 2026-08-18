package org.team100.lib.trajectory.se2.constraint;

import org.team100.lib.path.se2.PathSE2Point;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Mecanum drive has a diamond-shaped velocity envelope. If the x and y
 * directions responded the same (they don't), it would be a square.
 * 
 * This ignores the interaction with rotation.
 */
public class DiamondConstraint implements TimingConstraint {
    /** Max velocity ahead */
    private final double m_maxVelocityX;
    /** Max velocity to the side */
    private final double m_maxVelocityY;
    private final double m_maxAccel;

    /**
     * @param parent log
     * @param maxVX  max velocity straight ahead, typically higher
     * @param maxVY  max velocity sideways, typically lower
     * @param maxA   accel
     */
    public DiamondConstraint(double maxVX, double maxVY, double maxA) {
        m_maxVelocityX = maxVX;
        m_maxVelocityY = maxVY;
        m_maxAccel = maxA;
    }

    @Override
    public double maxV(PathSE2Point point) {
        Rotation2d course = point.waypoint().course().toRotation();
        Rotation2d heading = point.waypoint().pose().getRotation();
        Rotation2d strafe = course.minus(heading);
        // a rhombus is a superellipse with exponent 1
        // https://en.wikipedia.org/wiki/Superellipse
        double a = m_maxVelocityX;
        double b = m_maxVelocityY;
        return 1 / (Math.abs(strafe.getCos() / a) + Math.abs(strafe.getSin() / b));
    }

    @Override
    public double maxAccel(PathSE2Point point, double velocityM_S) {
        return m_maxAccel;
    }

    @Override
    public double maxDecel(PathSE2Point point, double velocity) {
        return -m_maxAccel;
    }

}
