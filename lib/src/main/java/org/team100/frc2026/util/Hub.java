package org.team100.frc2026.util;

import java.util.Optional;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;

public class Hub {
    /**
     * Can we score currently?
     * 
     * See 2026 rule 6.4.1.
     */
    public static boolean active() {
        Optional<Shift> currentShift = Shift.current();
        if (currentShift.isEmpty())
            return false;
        Optional<Alliance> alliance = MatchState.getAlliance();
        if (alliance.isEmpty())
            return false;
        return currentShift.get().active(alliance.get());
    }
}
