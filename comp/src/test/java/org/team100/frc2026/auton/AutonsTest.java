package org.team100.frc2026.auton;

import org.junit.jupiter.api.Test;
import org.team100.frc2026.robot.Machinery;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

public class AutonsTest {
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TotalCurrentLog currentLog = new TotalCurrentLog(log);

    @Test
    void testAll() {
        try (
                Machinery machinery = new Machinery(log, log, currentLog);
                Autons autons = new Autons(log, machinery)) {
        }
    }

}
