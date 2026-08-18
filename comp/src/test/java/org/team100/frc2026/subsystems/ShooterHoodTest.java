package org.team100.frc2026.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;

import org.junit.jupiter.api.Test;
import org.team100.frc2026.Timeless2026;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterHoodTest implements Timeless2026 {
    private static final double DELTA = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TotalCurrentLog currentLog = new TotalCurrentLog(log);

    @Test
    void test0() {
        ShooterHood hood = new ShooterHood(log, currentLog, () -> OptionalDouble.empty());
        // Mech starts at zero.
        assertEquals(0, hood.getUnwrappedPositionRad(), DELTA);
        // Goal starts at measurement.
        assertEquals(0, hood.getUnwrappedGoal().x(), DELTA);
        Command position = hood.autoPosition();
        position.initialize();
        position.execute();
        // Out-of-bounds means there is no goal.
        assertNull(hood.getUnwrappedGoal());
        // Position has not moved
        assertEquals(0, hood.getUnwrappedPositionRad(), DELTA);
        // Not on target; there is no valid target.
        assertFalse(hood.onTarget());
    }

    @Test
    void test1() {
        // Goal needs to be less than max position
        ShooterHood hood = new ShooterHood(log, currentLog, () -> OptionalDouble.of(0.4));
        // Mech starts at zero.
        assertEquals(0, hood.getUnwrappedPositionRad(), DELTA);
        // Goal starts at measurement.
        assertEquals(0, hood.getUnwrappedGoal().x(), DELTA);
        Command position = hood.autoPosition();
        position.initialize();
        position.execute();
        // Goal is OK
        assertEquals(0.4, hood.getUnwrappedGoal().x(), DELTA);
        for (int i = 0; i < 10; ++i) {
            stepTime();
            position.execute();
            hood.periodic();
        }
        // partway there
        assertEquals(0.292, hood.getUnwrappedPositionRad(), DELTA);
        for (int i = 0; i < 10; ++i) {
            stepTime();
            position.execute();
            hood.periodic();
        }
        // all the way there
        assertEquals(0.4, hood.getUnwrappedPositionRad(), DELTA);
        // finally on target
        assertTrue(hood.onTarget());
    }

}
