package org.team100.lib.reference.se2;

import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateSE2;

/**
 * Always returns the goal.
 */
public class ConstantReferenceSE2 implements ReferenceSE2 {
    private final StateSE2 m_goal;

    public ConstantReferenceSE2(StateSE2 goal) {
        m_goal = goal;
    }

    @Override
    public void initialize(StateSE2 measurement) {
        //
    }

    @Override
    public StateSE2 current() {
        return m_goal;
    }

    @Override
    public ControlSE2 next() {
        return m_goal.control();
    }

    /** TODO: this looks wrong, should be true? */
    @Override
    public boolean done() {
        return false;
    }

    @Override
    public StateSE2 goal() {
        return m_goal;
    }

    /** This reference is always at the goal. */
    @Override
    public double toGo() {
        return 0.0;
    }

}
