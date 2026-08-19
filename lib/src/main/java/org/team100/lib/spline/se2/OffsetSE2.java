package org.team100.lib.spline.se2;

import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.PathUtil;
import org.team100.lib.path.se2.PathSE2Entry;
import org.team100.lib.path.se2.PathSE2Parameter;
import org.team100.lib.path.se2.PathSE2Point;

import org.wpilib.math.linalg.Vector;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.numbers.N2;

/**
 * A path defined as an offset from a spline.
 * 
 * The length of the offset is fixed; the angle is the reciprocal of the
 * heading.
 * 
 * The use-case is to define the spline as the toolpoint, and the offset as
 * whatever is carrying the toolpoint around, e.g. the drivetrain.
 * 
 * That way, we can do three things correctly:
 * 
 * * "drawings" of the toolpoint spline, using the real work site
 * * optimization of the trajectory schedule of the drivetrain
 * * runtime feedback using the toolpoint, not the drivetrain.
 * 
 */
public class OffsetSE2 implements ISplineSE2 {
    private static final double DEFAULT_SCALE = 1;

    private final SplineSE2 m_toolpoint;
    private final double m_length;

    public OffsetSE2(SplineSE2 toolpoint, double length) {
        m_toolpoint = toolpoint;
        m_length = length;
    }

    @Override
    public WaypointSE2 waypoint(double s) {
        return new WaypointSE2(pose(s), course(s), DEFAULT_SCALE);
    }

    @Override
    public Pose2d pose(double s) {
        Vector<N2> p = SplineSE2Util.offsetR(m_toolpoint, m_length, s);
        double x = p.get(0);
        double y = p.get(1);
        Rotation2d heading = heading(s);
        return new Pose2d(x, y, heading);
    }

    private Rotation2d heading(double s) {
        return m_toolpoint.pose(s).getRotation();
    }

    @Override
    public Vector<N2> K(double s) {
        return PathUtil.K(rprime(s), rprimeprime(s));
    }

    @Override
    public PathSE2Entry entry(double s) {
        return new PathSE2Entry(parameter(s), point(s));
    }

    public PathSE2Parameter parameter(double s) {
        return new PathSE2Parameter(this, s);
    }

    public PathSE2Point point(double s) {
        return new PathSE2Point(waypoint(s), K(s));
    }

    private DirectionSE2 course(double s) {
        Vector<N2> rprime = rprime(s);
        double dx = rprime.get(0);
        double dy = rprime.get(1);
        double dheading = dheading(s);
        return new DirectionSE2(dx, dy, dheading);
    }

    private double dheading(double s) {
        // offset heading is always the same as the toolpoint
        return m_toolpoint.dheading(s);
    }

    /** First derivative of cartesian position wrt the parameter, s. */
    private Vector<N2> rprime(double s) {
        return m_toolpoint.rprime(s).plus(
                SplineSE2Util.offsetRprime(m_toolpoint, m_length, s));
    }

    /** Second derivative of cartesian position wrt the parameter, s. */
    private Vector<N2> rprimeprime(double s) {
        return m_toolpoint.rprimeprime(s).plus(
                SplineSE2Util.offsetRprimeprime(m_toolpoint, m_length, s));
    }

}
