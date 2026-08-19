package org.team100.lib.trajectory.se2;

import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.util.Math100;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class TrajectorySE2Point {
    private static final boolean DEBUG = false;
    /**
     * Point on a path, with heading rate and curvature.
     */
    private final PathSE2Point m_point;
    /**
     * Time we achieve this state.
     */
    private final double m_timeS;
    /**
     * Instantaneous pathwise velocity, m/s.
     */
    private final double m_velocityM_S;
    /**
     * Pathwise acceleration for the timespan after this state, m/s^2. It's computed
     * by looking at the velocity of the next state, and the distance to get there.
     * NOTE: *PATHWISE ONLY* not centrifugal acceleration.
     */
    private final double m_accelM_S_S;

    public TrajectorySE2Point(
            PathSE2Point point,
            double t,
            double velocity,
            double acceleration) {
        m_point = point;
        m_timeS = t;
        m_velocityM_S = velocity;
        m_accelM_S_S = acceleration;
    }

    /** path point */
    public PathSE2Point point() {
        return m_point;
    }

    /** Instant this point is reached, seconds */
    public double time() {
        return m_timeS;
    }

    /** Instantaneous pathwise velocity, m/s */
    public double velocity() {
        return m_velocityM_S;
    }

    /** Instantaneous pathwise (not centripetal) acceleration, m/s^2 */
    public double accel() {
        return m_accelM_S_S;
    }

    @Override
    public String toString() {
        return String.format("TrajectorySE2Point [ %s,  %5.3f, %5.3f, %5.3f ]",
                point(), time(), velocity(), accel());
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof TrajectorySE2Point)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }
        TrajectorySE2Point ts = (TrajectorySE2Point) other;
        if (!point().equals(ts.point())) {
            if (DEBUG)
                System.out.println("wrong state");
            return false;
        }
        if (!Math100.epsilonEquals(time(), ts.time())) {
            if (DEBUG)
                System.out.println("wrong time");
            return false;
        }
        return true;
    }

    /**
     * Control required for this point.
     * 
     * Correctly computes centripetal acceleration.
     */
    public ControlSE2 control() {
        WaypointSE2 waypoint = m_point.waypoint();
        Pose2d pose = waypoint.pose();
        DirectionSE2 course = waypoint.course();

        // Pose.
        double pX = pose.getTranslation().getX();
        double pY = pose.getTranslation().getY();
        double pTheta = pose.getRotation().getRadians();

        // Velocity.
        Rotation2d R = course.toRotation();
        double velX = R.getCos() * m_velocityM_S;
        double velY = R.getSin() * m_velocityM_S;
        double velTheta = course.headingRate() * m_velocityM_S;

        // Pathwise acceleration.
        double pathAX = R.getCos() * m_accelM_S_S;
        double pathAY = R.getSin() * m_accelM_S_S;
        double pathATheta = course.headingRate() * m_accelM_S_S;

        // Centripetal acceleration.
        // a = v^2/r = v^2 * curvature
        // this works because the acceleration vector is always normal
        // to the course vector, and in 2d, with signed curvature, that
        // determines the vector.
        double curvatureRad_M = m_point.k();
        double centripetalAccelM_s_s = m_velocityM_S * m_velocityM_S * curvatureRad_M;
        double centripetalAX = -1.0 * R.getSin() * centripetalAccelM_s_s;
        double centripetalAY = R.getCos() * centripetalAccelM_s_s;

        return new ControlSE2(
                new ControlR1(pX, velX, pathAX + centripetalAX),
                new ControlR1(pY, velY, pathAY + centripetalAY),
                new ControlR1(pTheta, velTheta, pathATheta));
    }

}
