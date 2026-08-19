package org.team100.lib.commands;

import java.util.Map;
import java.util.Optional;

import org.wpilib.command2.Command;
import org.wpilib.command2.PrintCommand;
import org.wpilib.command2.SelectCommand;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;

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
        Optional<Alliance> opt = MatchState.getAlliance();
        if (opt.isEmpty())
            return Select.UNKNOWN;
        switch (opt.get()) {
            case RED:
                return Select.RED;
            case BLUE:
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
