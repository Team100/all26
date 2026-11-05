package org.team100.lib.reference.r1;

/** For when you won't be using the reference, i.e. only "direct" actuation. */

public class NoVelocityReferenceR1 implements VelocityReferenceR1 {

    @Override
    public void setGoal(double goal) {
    }

    @Override
    public void init(double measurement) {
    }

    @Override
    public VelocitySetpointsR1 get() {
        return null;
    }

    @Override
    public boolean profileDone() {
        return false;
    }

    @Override
    public boolean valid() {
        return false;
    }

}
