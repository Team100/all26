package org.team100.frc2025.CommandGroups;

import static org.wpilib.command2.Commands.sequence;

import java.util.function.Supplier;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;

import org.wpilib.command2.Command;

/** Intake algae and hold it forever. */
public class MoveToAlgaePosition {
    public static Command get(
            CalgamesMech mech,
            Supplier<ScoringLevel> level,
            Supplier<Boolean> buttons) {
        Command pick = mech.algaeReefPick(level);
        Command exit = mech.algaeReefExit(level);
        return sequence(
                pick.until(() -> !buttons.get()), exit);
    }
}
