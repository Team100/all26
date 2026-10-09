package org.team100.frc2026.util;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Describes the "shifts" in the 2026 game, and which one is "active" based on
 * the auton winner.
 * 
 * see https://gist.github.com/LordOfFrogs/240ba37cf696ba156d87f387c1461bd5
 */
public enum Shift {
    AUTO(0, 20, ActiveType.BOTH),
    TRANSITION(20, 30, ActiveType.BOTH),
    SHIFT_1(30, 55, ActiveType.AUTO_LOSER),
    SHIFT_2(55, 80, ActiveType.AUTO_WINNER),
    SHIFT_3(80, 105, ActiveType.AUTO_LOSER),
    SHIFT_4(105, 130, ActiveType.AUTO_WINNER),
    ENDGAME(130, 160, ActiveType.BOTH);

    final int startTime;
    final int endTime;
    final ActiveType activeType;

    /** Return the current shift, or empty if no valid shift */
    static Optional<Shift> current() {
        OptionalDouble matchTime = MatchTime.get();
        if (matchTime.isEmpty())
            return Optional.empty();

        for (Shift shift : Shift.values()) {
            if (matchTime.getAsDouble() < shift.endTime) {
                return Optional.of(shift);
            }
        }
        return Optional.empty();
    }

    /** Is this shift active, given the auto winner? */
    boolean active(Alliance alliance) {
        Optional<Alliance> autoWinner = AutoWinner.get();
        return switch (activeType) {
            case BOTH -> true;
            case AUTO_WINNER -> autoWinner.isPresent() && autoWinner.get() == alliance;
            case AUTO_LOSER -> autoWinner.isPresent() && autoWinner.get() != alliance;
            default -> false;
        };
    }

    private Shift(int startTime, int endTime, ActiveType activeType) {
        this.startTime = startTime;
        this.endTime = endTime;
        this.activeType = activeType;
    }

    private enum ActiveType {
        BOTH,
        AUTO_WINNER,
        AUTO_LOSER
    }
}
