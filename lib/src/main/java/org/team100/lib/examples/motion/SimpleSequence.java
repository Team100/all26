package org.team100.lib.examples.motion;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;

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
