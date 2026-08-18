package org.team100.lib.commands;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;

/**
 * Executes the red or blue command based on the current alliance.
 */
public class AllianceCommand extends SelectCommand<AllianceCommand.Select> {
    /**
     * We can't use null as a selector output so make an explicit unknown.
     */
    public enum Select {
        RED,
        BLUE,
        UNKNOWN
    }

    public AllianceCommand(Command red, Command blue) {
        super(Map.of(
                Select.RED, red,
                Select.BLUE, blue,
                Select.UNKNOWN, err()),
                AllianceCommand::selector);
    }

    private static Select selector() {
        Optional<Alliance> opt = DriverStation.getAlliance();
        if (opt.isEmpty())
            return Select.UNKNOWN;
        switch (opt.get()) {
            case Red:
                return Select.RED;
            case Blue:
                return Select.BLUE;
            default:
                return Select.UNKNOWN;
        }
    }

    private static Command err() {
        // each instance gets its own error printer to avoid tripping
        // the multi-composition detector
        return new PrintCommand("AllianceCommand: no alliance!");
    }
}
