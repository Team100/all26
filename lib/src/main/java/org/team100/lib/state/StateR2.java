package org.team100.lib.state;

import org.team100.lib.geometry.r2.VelocityR2;
import org.wpilib.math.geometry.Translation2d;

/** Represents planar position only. */
public class StateR2 {
    private final StateR1 m_x;
    private final StateR1 m_y;

    public StateR2(StateR1 x, StateR1 y) {
        m_x = x;
        m_y = y;
    }

    public StateR2(Translation2d x, VelocityR2 v) {
        this(
                new StateR1(x.getX(), v.x()),
                new StateR1(x.getY(), v.y()));
    }

    /** Motionless with the specified pose */
    public StateR2(Translation2d x) {
        this(x, VelocityR2.ZERO);
    }

    /** Motionless at the origin */
    public StateR2() {
        this(new StateR1(), new StateR1());
    }

    public ControlR2 control() {
        return new ControlR2(m_x.control(), m_y.control());
    }

    /** Component-wise difference (not geodesic) */
    public StateR2 minus(StateR2 other) {
        return new StateR2(x().minus(other.x()), y().minus(other.y()));
    }

    /** Component-wise sum (not geodesic) */
    public StateR2 plus(StateR2 other) {
        return new StateR2(x().plus(other.x()), y().plus(other.y()));
    }

    /**
     * Use the current velocity to evolve the position of each dimension
     * independently.
     * 
     * This does not describe geodesic paths in SE(2). For that, see Twist2d.
     */
    public StateR2 evolve(double dt) {
        return new StateR2(m_x.evolve(dt), m_y.evolve(dt));
    }

    /** All dimensions position and velocity are within (the same) tolerance */
    public boolean near(StateR2 other, double tolerance) {
        return x().near(other.x(), tolerance)
                && y().near(other.y(), tolerance);
    }

    /** Translation of the pose. */
    public Translation2d translation() {
        return new Translation2d(m_x.x(), m_y.x());
    }

    public VelocityR2 velocityR2() {
        return new VelocityR2(m_x.v(), m_y.v());
    }

    public StateR1 x() {
        return m_x;
    }

    public StateR1 y() {
        return m_y;
    }

    public String toString() {
        return "ModelR2(" + m_x + ", " + m_y + ")";
    }

}
