package org.team100.lib.state;

import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;
import org.wpilib.math.geometry.Translation2d;

/** Represents planar position only. */
public class ControlR2 {

      private final ControlR1 m_x;
    private final ControlR1 m_y;

    public ControlR2(ControlR1 x, ControlR1 y) {
        m_x = x;
        m_y = y;
    }

    public ControlR2(Translation2d x, VelocityR2 v) {
        this(
                new ControlR1(x.getX(), v.x(), 0),
                new ControlR1(x.getY(), v.y(), 0));
    }

    public ControlR2(Translation2d x, VelocityR2 v, AccelerationR2 a) {
        this(
                new ControlR1(x.getX(), v.x(), a.x()),
                new ControlR1(x.getY(), v.y(), a.y()));
    }

    public ControlR2(Translation2d x) {
        this(x, VelocityR2.ZERO);
    }

 

    public static ControlR2 zero() {
        return new ControlR2(new ControlR1(), new ControlR1());
    }

    public StateR2 model() {
        return new StateR2(m_x.model(), m_y.model());
    }

    /** Component-wise difference (not geodesic) */
    public ControlR2 minus(ControlR2 other) {
        return new ControlR2(x().minus(other.x()), y().minus(other.y()));
    }

    /** Component-wise sum (not geodesic) */
    public ControlR2 plus(ControlR2 other) {
        return new ControlR2(x().plus(other.x()), y().plus(other.y()));
    }

    public boolean near(ControlR2 other, double tolerance) {
        return x().near(other.x(), tolerance)
                && y().near(other.y(), tolerance);
    }


    /** Translation of the pose */
    public Translation2d translation() {
        return new Translation2d(m_x.x(), m_y.x());
    }


    public VelocityR2 velocity() {
        return new VelocityR2(m_x.v(), m_y.v());
    }

    public AccelerationR2 acceleration() {
        return new AccelerationR2(m_x.a(), m_y.a());
    }

    public ControlR1 x() {
        return m_x;
    }

    public ControlR1 y() {
        return m_y;
    }


    public String toString() {
        return "ControlR2(" + m_x + ", " + m_y + ")";
    }

    
}
