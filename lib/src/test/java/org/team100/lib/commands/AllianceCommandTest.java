package org.team100.lib.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.InstantCommand;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

class AllianceCommandTest {
    private String output = "none";

    @BeforeEach
    void init() {
        HAL.initialize(500, 0);
    }

    @Test
    void testInvalid() {
        AllianceCommand c = new AllianceCommand(
                new InstantCommand(() -> output = "red"),
                new InstantCommand(() -> output = "blue"));
        DriverStationSim.setAllianceStationId(AllianceStationID.UNKNOWN);
        DriverStationSim.notifyNewData();
        assertTrue(MatchState.getAlliance().isEmpty());
        assertEquals("none", output);
        c.initialize();
        assertEquals("none", output);
    }

    @Test
    void testRed() {
        AllianceCommand c = new AllianceCommand(
                new InstantCommand(() -> output = "red"),
                new InstantCommand(() -> output = "blue"));
        DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
        DriverStationSim.notifyNewData();
        assertEquals(Alliance.RED, MatchState.getAlliance().get());
        assertEquals("none", output);
        c.initialize();
        assertEquals("red", output);
    }

    @Test
    void testBlue() {
        AllianceCommand c = new AllianceCommand(
                new InstantCommand(() -> output = "red"),
                new InstantCommand(() -> output = "blue"));
        DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
        DriverStationSim.notifyNewData();
        assertEquals(Alliance.BLUE, MatchState.getAlliance().get());
        assertEquals("none", output);
        c.initialize();
        assertEquals("blue", output);
    }
}
