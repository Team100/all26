package org.team100.lib.subsystems.shooter;

import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;

public interface DualDrumShooter extends Subsystem {
    /** Runs forever */
    Command spinSlow();

    /** Runs forever */
    Command spinFast();

    /** Runs forever. */
    Command stop();

    boolean atGoal();
}
