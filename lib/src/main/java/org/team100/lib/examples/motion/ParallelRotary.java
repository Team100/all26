package org.team100.lib.examples.motion;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Illustrates two subsystems that need to act at the same time, until they're
 * both done.
 */
public class ParallelRotary {
    public static Command get(
            LoggerFactory log,
            RotaryPositionSubsystem1d r1,
            RotaryPositionSubsystem1d r2) {
        Command c1 = r1.goToTheSpot();
        Command c2 = r2.goToTheSpot();
        return parallel(c1, c2).until(() -> r1.isDone() && r2.isDone());
    }
}
