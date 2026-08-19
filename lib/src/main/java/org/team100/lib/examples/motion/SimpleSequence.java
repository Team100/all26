package org.team100.lib.examples.motion;

import static org.wpilib.command2.Commands.sequence;

import org.team100.lib.logging.LoggerFactory;

import org.wpilib.command2.Command;

/**
 * This illustrates a short sequence, contained within a separate file, instead
 * of inlined in the Robot Container. If you put *everything* in the container,
 * it can get unwieldy, so making little classes like this helps keep things
 * tidy.
 */
public class SimpleSequence {
    public static Command get(
            LoggerFactory log,
            RotaryPositionSubsystem1d rotary,
            OpenLoopSubsystem openloop) {
        return sequence(
                rotary.goToTheSpot().until(rotary::isDone),
                openloop.forward().withTimeout(2),
                rotary.goHome().until(rotary::isDone));
    }
}
