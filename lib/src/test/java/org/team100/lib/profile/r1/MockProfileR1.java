package org.team100.lib.profile.r1;

import org.team100.lib.state.ControlR1;
import org.team100.lib.state.StateR1;

public class MockProfileR1 implements ProfileR1 {
    ControlR1 result;
    double eta;
    int count = 0;

    @Override
    public ControlR1 calculate(double dt, ControlR1 initial, StateR1 goal) {
        count++;
        return result;
    }

    @Override
    public MockProfileR1 scale(double s) {
        return this;
    }

    public double getMaxVelocity() {
        return 0;
    }

    @Override
    public double solve(double dt, ControlR1 i, StateR1 g, double eta, double etaTolerance) {
        return 1.0;
    }

}
