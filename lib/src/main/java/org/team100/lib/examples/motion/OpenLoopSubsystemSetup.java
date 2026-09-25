package org.team100.lib.examples.motion;

import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;

public class OpenLoopSubsystemSetup {
    public OpenLoopSubsystemSetup(LoggerFactory log, TotalCurrentLog currentLog, DriverXboxControl control) {
        OpenLoopSubsystem openloop = new OpenLoopSubsystem(log, currentLog);

        /**
         * Binds some buttons to the open loop system.
         */
        whileTrue(control::a,
                openloop.forward());
        whileTrue(control::b,
                openloop.reverse());
    }
}
