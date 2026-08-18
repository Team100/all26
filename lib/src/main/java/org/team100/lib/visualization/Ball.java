package org.team100.lib.visualization;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface Ball {
    /** Sets initial position and velocity. */
    void launch();

    /** Evolves state one time step. */
    void fly();

    void reset();

    void periodic();

    /** Shoot the ball and continue its path as long as the command runs. */
    default Command shoot() {
        return Commands.startRun(this::launch, this::fly)
                .finallyDo(this::reset);
    }

}
