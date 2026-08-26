package org.team100.lib.reference.se2;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.ControlSE2Logger;
import org.team100.lib.logging.LoggerFactory.StateSE2Logger;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateSE2;

/**
 * Produces references based on profiles.
 * 
 * This uses the central Cache to manage reference updates, because it's
 * important that the reference updates be aligned with the clock, i.e. one
 * profile update step per Takt step, but the SwerveReference interface doesn't
 * know anything about time. Still, it's a sort of strange thing to do,
 * since it's not a "Cache" per se.
 * 
 * One of the implications of this choice is that the cache updates will
 * continue forever unless we explicitly stop them, so if you use this class,
 * you need to remember to do that.
 */
public class ProfileReferenceSE2 implements ReferenceSE2 {
    private static final boolean DEBUG = false;
    private static final double TOLERANCE = 0.01;

    /**
     * Putting these in the same class allows us to refresh them both atomically.
     */
    private record References(StateSE2 m_current, ControlSE2 m_next) {
    }

    private final LoggerFactory m_log;

    private final ProfileSE2 m_profile;
    /** The name is for debugging. */
    private final String m_name;
    private final ObjectCache<References> m_references;
    private final StateSE2Logger m_log_current;
    private final ControlSE2Logger m_log_next;
    private final BooleanLogger m_log_done;
    private final StateSE2Logger m_log_goal;

    private StateSE2 m_goal;
    private boolean m_done;

    /**
     * The most-recently calculated "next" reference, which will be a future
     * "current" reference.
     */
    private ControlSE2 m_next;

    public ProfileReferenceSE2(
            LoggerFactory parent,
            ProfileSE2 profile,
            String name) {
        m_log = parent.type(this);
        m_profile = profile;
        m_name = name;
        // this will keep polling until we stop it.
        m_references = Cache.of(() -> refresh(m_next == null ? null : m_next.model()));

        m_log_current = m_log.StateSE2Logger(Level.TRACE, "current");
        m_log_next = m_log.controlSE2Logger(Level.TRACE, "next");
        m_log_done = m_log.booleanLogger(Level.TRACE, "done");
        m_log_goal = m_log.StateSE2Logger(Level.TRACE, "goal");
    }

    /**
     * This does not solve for profile coordination, so if you update the goal after
     * initialize(), you'll use the old scales. That's probably fine, if the goal
     * hasn't moved much, but it's not appropriate to move the goal a lot.
     */
    public void setGoal(StateSE2 goal) {
        m_goal = goal;
    }

    /** Immediately overwrite the references. */
    @Override
    public void initialize(StateSE2 measurement) {
        m_profile.solve(measurement, m_goal);
        m_references.set(refresh(measurement));
        m_done = false;
    }

    @Override
    public StateSE2 current() {
        StateSE2 current = m_references.get().m_current;
        m_log_current.log(() -> current);
        return current;
    }

    @Override
    public ControlSE2 next() {
        ControlSE2 next = m_references.get().m_next;
        m_log_next.log(() -> next);
        return next;
    }

    /** Setpoint is near the goal. */
    @Override
    public boolean done() {
        m_log_done.log(() -> m_done);
        return m_done;
    }

    @Override
    public StateSE2 goal() {
        m_log_goal.log(() -> m_goal);
        return m_goal;
    }

    /**
     * Stop updating the references. There's no way to restart the updates, so you
     * should discard this object once you call end().
     */
    public void end() {
        m_references.end();
    }

    ////////////////////////////////////

    private References refresh(StateSE2 newCurrent) {
        m_next = makeNext(newCurrent);
        return new References(newCurrent, m_next);
    }

    private ControlSE2 makeNext(StateSE2 current) {
        if (DEBUG) {
            System.out.printf("ProfileReference refreshing %s\n", m_name);
        }
        if (current == null) {
            // happens at startup
            return null;
        }
        if (m_goal == null) {
            return current.control();
        }
        ControlSE2 next = m_profile.calculate(current, m_goal);
        if (current.near(m_goal, TOLERANCE))
            m_done = true;
        return next;
    }

}
