package org.team100.lib.subsystems.rn;

import java.util.List;

import org.team100.lib.state.ControlR1;
import org.team100.lib.state.StateR1;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.util.Num;

/** Represents position in joint space ("Q") with N independent dimensions. */
public interface PositionSubsystemRn<N extends Num> extends Subsystem {
    void setRn(List<ControlR1> setpoint);

    List<StateR1> getStateRn();

    void stop();
}
