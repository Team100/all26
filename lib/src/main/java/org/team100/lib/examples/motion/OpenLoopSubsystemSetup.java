package org.team100.lib.examples.motion;

import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OpenLoopSubsystemSetup {
    public OpenLoopSubsystemSetup(LoggerFactory log, TotalCurrentLog currentLog, DriverXboxControl control) {
        OpenLoopSubsystem openloop = new OpenLoopSubsystem(log, currentLog);

        /**
         * Binds some buttons to the open loop system.
         */
        new Trigger(control::a).whileTrue(openloop.forward());
        new Trigger(control::b).whileTrue(openloop.reverse());
    }
}
