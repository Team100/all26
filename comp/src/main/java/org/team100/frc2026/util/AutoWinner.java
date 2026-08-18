package org.team100.frc2026.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The FMS tells us who won auto, so we can deduce whose shift it is.
 * 
 * see https://gist.github.com/LordOfFrogs/240ba37cf696ba156d87f387c1461bd5
 */
public class AutoWinner {
    /**
     * Returns the Alliance that won auto as specified by the FMS/Driver
     * Station's game specific message data, or empty if no game message or alliance
     * is available.
     */
    public static Optional<Alliance> get() {
        String msg = DriverStation.getGameSpecificMessage();
        char msgChar = msg.length() > 0 ? msg.charAt(0) : ' ';
        switch (msgChar) {
            case 'B':
                return Optional.of(Alliance.Blue);
            case 'R':
                return Optional.of(Alliance.Red);
            default:
                return Optional.empty();
        }
    }

}
