package org.team100.lib.examples.motion;

import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;

public class ParallelRotarySetup {

    public ParallelRotarySetup(
            LoggerFactory log, TotalCurrentLog currentLog, DriverXboxControl control) {
        RotaryPositionSubsystem1d r1 = new RotaryPositionSubsystem1d(log, currentLog);
        RotaryPositionSubsystem1d r2 = new RotaryPositionSubsystem1d(log, currentLog);
        whileTrue(control::a,
                ParallelRotary.get(log, r1, r2));
    }
}
