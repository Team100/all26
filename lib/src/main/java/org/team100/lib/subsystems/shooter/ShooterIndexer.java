package org.team100.lib.subsystems.shooter;

import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;

/**
 * For experimenting with control methods for shooter indexing.
 */
public interface ShooterIndexer extends Subsystem {
    /** End when done, so you can trigger with "onTrue". */
    Command single();

    /** Runs forever. */
    Command continuous();

    /** Runs forever. */
    Command stop();
}
