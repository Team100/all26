package org.team100.lib.examples.motion;

import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CombinedSetup {
    public CombinedSetup(
            LoggerFactory log, TotalCurrentLog currentLog, DriverXboxControl control) {
        RotaryPositionSubsystem1d rotary = new RotaryPositionSubsystem1d(log, currentLog);
        OpenLoopSubsystem openloop = new OpenLoopSubsystem(log, currentLog);

        /**
         * Illustrates a short sequence: move, roll for 2 sec, move back.
         */
        new Trigger(control::rightTrigger).whileTrue(SimpleSequence.get(log, rotary, openloop));
    }
}
