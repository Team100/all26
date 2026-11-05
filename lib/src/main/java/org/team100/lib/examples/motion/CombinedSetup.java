package org.team100.lib.examples.motion;

import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;

public class CombinedSetup {
    public CombinedSetup(
            LoggerFactory log, TotalCurrentLog currentLog, DriverXboxControl control) {
        RotaryPositionSubsystem1d rotary = new RotaryPositionSubsystem1d(log, currentLog);
        OpenLoopSubsystem openloop = new OpenLoopSubsystem(log, currentLog);

        /**
         * Illustrates a short sequence: move, roll for 2 sec, move back.
         */
        whileTrue(control::rightTrigger,
                SimpleSequence.get(log, rotary, openloop));
    }
}
