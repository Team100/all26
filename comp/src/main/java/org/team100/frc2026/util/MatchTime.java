package org.team100.frc2026.util;

import java.util.OptionalDouble;

import org.wpilib.framework.RobotBase;
import org.wpilib.system.Timer;

/**
 * Counts up from 0 to 160 seconds, or empty if match isn't running, or if
 * between auto and teleop.
 * 
 * see https://gist.github.com/LordOfFrogs/240ba37cf696ba156d87f387c1461bd5
 */
public class MatchTime {
    public static OptionalDouble get() {
        double matchTime = Timer.getMatchTime();
        if (matchTime < 0)
            return OptionalDouble.empty();
        if (RobotBase.isAutonomous()) {
            return OptionalDouble.of(20 - matchTime);
        }
        if (RobotBase.isTeleop()) {
            return OptionalDouble.of(160 - matchTime);
        }
        return OptionalDouble.empty();
    }
}
