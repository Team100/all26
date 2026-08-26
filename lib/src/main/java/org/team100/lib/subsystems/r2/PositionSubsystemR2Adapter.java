package org.team100.lib.subsystems.r2;

import org.team100.lib.state.ControlR2;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateR1;
import org.team100.lib.state.StateR2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;

/** Adapt an R2 subsystem to SE2, ignoring rotation. */
public class PositionSubsystemR2Adapter implements PositionSubsystemSE2 {

    PositionSubsystemR2 s;

    public PositionSubsystemR2Adapter(PositionSubsystemR2 s) {
        this.s = s;
    }

    @Override
    public StateSE2 getState() {
        StateR2 m = s.getState();
        return new StateSE2(m.x(), m.y(), new StateR1());
    }

    @Override
    public void stop() {
        s.stop();
    }

    @Override
    public void set(ControlSE2 setpoint) {
        s.set(new ControlR2(setpoint.x(), setpoint.y()));
    }

}
