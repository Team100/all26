package org.team100.lib.reference.se2;

import java.util.function.Supplier;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.ControlSE2Logger;
import org.team100.lib.logging.LoggerFactory.StateSE2Logger;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Entry;
import org.team100.lib.trajectory.se2.TrajectorySE2Point;
import org.wpilib.math.geometry.Translation2d;

/**
 * Use the current position to choose trajectory references.
 * 
 * First finds the geometrically closest point on the trajectory, then samples a
 * bit into the future from that point. If the position is always on the
 * trajectory, this should behave roughly the same as the the usual time-based
 * reference.
 * 
 * This only returns references on the trajectory, which means that if the
 * position is far from the trajectory, the controller task can easily become
 * infeasible.
 * 
 * This is a bit different (simpler) than the usual "pursuit" behavior, which
 * chooses references along a line towards a future trajectory point.
 * 
 * The velocity feedforward will be verbatim from the trajectory points, not
 * aimed at the nearest point. In effect, velocity feedforward is used to push
 * the robot toward the future state, and position feedback is used to get back
 * to the trajectory path.
 * 
 * The main purpose of this class is not as a general pursuit follower, but
 * rather as a timed follower that recovers gracefully from delay and
 * displacement, while still trying to stay on the path.
 * 
 * TODO: aim slightly further in the future.
 * TODO: produce references along the line to the trajectory, to reduce the
 * controller error.
 */
public class TrajectoryPursuitReferenceSE2 implements ReferenceSE2 {
    private final TrajectorySE2 m_trajectory;
    private final Supplier<Translation2d> m_position;
    private final StateSE2Logger m_log_current;
    private final ControlSE2Logger m_log_next;
    private final BooleanLogger m_log_done;
    private final StateSE2Logger m_log_goal;

    public TrajectoryPursuitReferenceSE2(
            LoggerFactory parent,
            TrajectorySE2 trajectory,
            Supplier<Translation2d> position) {
        LoggerFactory log = parent.type(this);
        m_trajectory = trajectory;
        m_position = position;
        m_log_current = log.StateSE2Logger(Level.TRACE, "current");
        m_log_next = log.controlSE2Logger(Level.TRACE, "next");
        m_log_done = log.booleanLogger(Level.TRACE, "done");
        m_log_goal = log.StateSE2Logger(Level.TRACE, "goal");
    }

    @Override
    public void initialize(StateSE2 measurement) {
        // nothing to initialize
    }

    @Override
    public StateSE2 current() {
        StateSE2 current = sample(tca()).state();
        m_log_current.log(() -> current);
        return current;
    }

    @Override
    public ControlSE2 next() {
        ControlSE2 next = sample(tca() + TimedRobot100.LOOP_PERIOD_S);
        m_log_next.log(() -> next);
        return next;
    }

    @Override
    public boolean done() {
        boolean done = m_trajectory.isDone(tca());
        m_log_done.log(() -> done);
        return done;
    }

    @Override
    public StateSE2 goal() {
        TrajectorySE2Entry entry = m_trajectory.getLastPoint();
        TrajectorySE2Point point = entry.point();
        StateSE2 goal = point.control().state();
        m_log_goal.log(() -> goal);
        return goal;
    }

    ////////////////////////////////////

    /** Time of closest approach */
    private double tca() {
        Translation2d p = m_position.get();
        double t = m_trajectory.timeOfClosestApproach(p);
        return t;
    }

    private ControlSE2 sample(double t) {
        TrajectorySE2Entry sample = m_trajectory.sample(t);
        TrajectorySE2Point point = sample.point();
        return point.control();
    }

}
