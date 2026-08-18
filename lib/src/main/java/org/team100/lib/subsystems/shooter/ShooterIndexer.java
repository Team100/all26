package org.team100.lib.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

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
