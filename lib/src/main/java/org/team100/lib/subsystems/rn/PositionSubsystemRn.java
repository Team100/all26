package org.team100.lib.subsystems.rn;

import java.util.List;

import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;
import org.wpilib.command2.Subsystem;

public interface PositionSubsystemRn extends Subsystem {
    void setRn(List<ControlR1> setpoint);

    List<ModelR1> getStateRn();

    void stop();
}
