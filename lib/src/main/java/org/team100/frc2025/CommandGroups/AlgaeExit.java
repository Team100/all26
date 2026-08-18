package org.team100.frc2025.CommandGroups;

import java.util.function.Supplier;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;

import edu.wpi.first.wpilibj2.command.Command;

/** Intake algae and hold it forever. */
public class AlgaeExit {
    public static Command get(
            CalgamesMech mech,
            Supplier<ScoringLevel> level) {

        Command pick = mech.algaeReefExit(level);
        return pick;
    }
}
