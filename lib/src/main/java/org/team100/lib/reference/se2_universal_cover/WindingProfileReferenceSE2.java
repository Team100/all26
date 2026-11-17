package org.team100.lib.reference.se2_universal_cover;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.WindingControlSE2Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.WindingStateSE2Logger;
import org.team100.lib.profile.se2_universal_cover.WindingProfileSE2;
import org.team100.lib.reference.se2.ReferenceSE2;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.WindingControlSE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.state.WindingStateSE2;

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
public class WindingProfileReferenceSE2 implements ReferenceSE2 {
    private static final boolean DEBUG = false;
    private static final double TOLERANCE = 0.01;

    /**
     * Putting these in the same class allows us to refresh them both atomically.
     */
    private record References(WindingStateSE2 m_current, WindingControlSE2 m_next) {
    }

    private final WindingProfileSE2 m_profile;
    /** The name is for debugging. */
    private final String m_name;
    private final ObjectCache<References> m_references;
    private final WindingStateSE2Logger m_log_current;
    private final WindingControlSE2Logger m_log_next;
    private final BooleanLogger m_log_done;
    private final WindingStateSE2Logger m_log_goal;
    private final DoubleLogger m_logToGo;

    private WindingStateSE2 m_goal;
    private boolean m_done;

    /**
     * The most-recently calculated "next" reference, which will be a future
     * "current" reference.
     */
    private WindingControlSE2 m_next;

    public WindingProfileReferenceSE2(
            LoggerFactory parent,
            WindingProfileSE2 profile,
            String name) {
        LoggerFactory log = parent.type(this);
        m_profile = profile;
        m_name = name;
        // this will keep polling until we stop it.
        m_references = Cache.of(() -> refresh(m_next == null ? null : m_next.state()));

        m_log_current = log.WindingStateSE2Logger(Level.TRACE, "current");
        m_log_next = log.WindingControlSE2Logger(Level.TRACE, "next");
        m_log_done = log.booleanLogger(Level.TRACE, "done");
        m_log_goal = log.WindingStateSE2Logger(Level.TRACE, "goal");
        m_logToGo = log.doubleLogger(Level.TRACE, "to go");
    }

    /**
     * This does not solve for profile coordination, so if you update the goal after
     * initialize(), you'll use the old scales. That's probably fine, if the goal
     * hasn't moved much, but it's not appropriate to move the goal a lot.
     */
    public void setGoal(WindingStateSE2 goal) {
        m_goal = goal;
    }

    /**
     * Immediately overwrite the references.
     * 
     * Uses the smallest preimage in ~SE(2), i.e. within [-pi,pi].
     * 
     * If you have ~SE(2) available (i.e. "unwrapped"), use that
     * instead.
     */
    @Override
    public void initialize(StateSE2 wrapped) {
        WindingStateSE2 unwrapped = new WindingStateSE2(wrapped);
        m_profile.solve(unwrapped, m_goal);
        m_references.set(refresh(unwrapped));
        m_done = false;
    }

    /** SE2 projection (i.e. "mod 2pi") */
    @Override
    public StateSE2 current() {
        WindingStateSE2 current = m_references.get().m_current;
        m_log_current.log(() -> current);
        return current.stateSE2();
    }

    /** SE2 projection (i.e. "mod 2pi") */
    @Override
    public ControlSE2 next() {
        WindingControlSE2 next = m_references.get().m_next;
        m_log_next.log(() -> next);
        return next.controlSE2();
    }

    /** Setpoint is near the goal. */
    @Override
    public boolean done() {
        m_log_done.log(() -> m_done);
        return m_done;
    }

    /** SE2 projection (i.e. "mod 2pi") */
    @Override
    public StateSE2 goal() {
        m_log_goal.log(() -> m_goal);
        return m_goal.stateSE2();
    }

    /** L2 norm of all components */
    @Override
    public double toGo() {
        double l2 = Metrics.l2Norm(m_goal.minus(m_references.get().m_current));
        m_logToGo.log(() -> l2);
        return l2;
    }

    /**
     * Stop updating the references. There's no way to restart the updates, so you
     * should discard this object once you call end().
     */
    public void end() {
        m_references.end();
    }

    ////////////////////////////

    private References refresh(WindingStateSE2 newCurrent) {
        m_next = makeNext(newCurrent);
        return new References(newCurrent, m_next);
    }

    private WindingControlSE2 makeNext(WindingStateSE2 current) {
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
        WindingControlSE2 next = m_profile.calculate(current, m_goal);
        if (current.near(m_goal, TOLERANCE))
            m_done = true;
        return next;
    }

}
