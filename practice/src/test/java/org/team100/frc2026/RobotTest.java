package org.team100.frc2026;

import org.junit.jupiter.api.Test;

/** This just checks for crashers. */
public class RobotTest {
    @Test
    void testSimulation() {
        Robot robot = new Robot();
        try {
            robot.robotInit();
            robot.simulationInit();
            robot.simulationPeriodic();
            robot.robotPeriodic();
        } finally {
            robot.close();
        }
    }

    @Test
    void testDisabled() {
        Robot robot = new Robot();
        try {
            robot.robotInit();
            robot.disabledPeriodic();
            robot.robotPeriodic();
        } finally {
            robot.close();
        }
    }

    @Test
    void testAutonomous() {
        Robot robot = new Robot();
        try {
            robot.robotInit();
            robot.autonomousInit();
            robot.autonomousPeriodic();
            robot.robotPeriodic();
        } finally {
            robot.close();
        }
    }

    @Test
    void testTeleop() {
        Robot robot = new Robot();
        try {
            robot.robotInit();
            robot.teleopInit();
            robot.teleopPeriodic();
            robot.robotPeriodic();
        } finally {
            robot.close();
        }
    }

    @Test
    void testTest() {
        Robot robot = new Robot();
        try {
            robot.robotInit();
            robot.testInit();
            robot.testPeriodic();
            robot.robotPeriodic();
        } finally {
            robot.close();
        }
    }
}
