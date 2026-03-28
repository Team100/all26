package org.team100.lib.logging;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

class LoggingTest {

    @BeforeEach
    void init() {
        HAL.initialize(500, 0);
        Logging.instance().resetForTest();
    }

    @AfterEach
    void cleanup() {
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.notifyNewData();
        Logging.instance().resetForTest();
    }

    @Test
    void testDefaultLevel() {
        assertEquals(Level.DEBUG, Logging.instance().getLevel());
    }

    @Test
    void testNoFmsReadsChooser() {
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.notifyNewData();
        Logging.instance().refreshLogLevel();
        // Default chooser selection is DEBUG
        assertEquals(Level.DEBUG, Logging.instance().getLevel());
    }

    @Test
    void testFmsAttachedLatchesToComp() {
        DriverStationSim.setFmsAttached(true);
        DriverStationSim.notifyNewData();
        Logging.instance().refreshLogLevel();
        assertEquals(Level.COMP, Logging.instance().getLevel());
    }

    @Test
    void testFmsLatchPersistsAfterDisconnect() {
        // Attach FMS and trigger latch
        DriverStationSim.setFmsAttached(true);
        DriverStationSim.notifyNewData();
        Logging.instance().refreshLogLevel();
        assertEquals(Level.COMP, Logging.instance().getLevel());

        // Disconnect FMS, level should remain COMP
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.notifyNewData();
        Logging.instance().refreshLogLevel();
        assertEquals(Level.COMP, Logging.instance().getLevel());
    }
}
