package org.team100.lib.reference.r1;

import org.team100.lib.state.StateR1;

/** For when you won't be using the reference, i.e. only "direct" actuation. */
public class NoReferenceR1 implements ReferenceR1 {

    @Override
    public void setGoal(StateR1 goal) {
    }

    @Override
    public void init(StateR1 measurement) {
    }

    @Override
    public SetpointsR1 get() {
        return null;
    }

    @Override
    public boolean profileDone() {
        return false;
    }

    @Override
    public boolean valid() {
        return true;
    }

}
