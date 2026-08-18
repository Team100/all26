package org.team100.lib.examples.motion;

import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is an example of what you'd put in RobotContainer to use some of the
 * subsystem and command classes here.
 */
public class RotaryPositionSubsystem1dSetup {

    public RotaryPositionSubsystem1dSetup(
            LoggerFactory log, TotalCurrentLog currentLog, DriverXboxControl control) {

        RotaryPositionSubsystem1d rotary = new RotaryPositionSubsystem1d(log, currentLog);

        /*
         * Binds the The trigger moves the subsystem to 1.5 with a profile.
         * 
         * Notice the pattern here:
         * 
         * * new trigger with a boolean, usually a controller button.
         * * "whileTrue" means run the command while you're holding the button
         * * the subsystem produces the command, which runs forever
         * * add a stop condition in "until"
         * 
         * Using an endless command with a separate stop condition is usually a good
         * choice: it avoids entangling the "stop" logic with the command itself. Note
         * in this case there's not even a Command class, it's just inline.
         * 
         * Note the use of "whileTrue" -- it cancels the command if you let go of the
         * button, which seems like a good idea.
         */
        new Trigger(control::x).whileTrue(rotary.goToTheSpot().until(rotary::isDone));
    }
}
