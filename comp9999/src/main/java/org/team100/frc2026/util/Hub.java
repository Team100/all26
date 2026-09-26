package org.team100.frc2026.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            return false;
        return currentShift.get().active(alliance.get());
    }
}
