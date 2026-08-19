// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.util.CanId;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private final KrakenX60Motor left;
    private final KrakenX60Motor right;

    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    private static final TotalCurrentLog currentLog = new TotalCurrentLog(rootLogger);

    public Robot() {
        m_robotContainer = new RobotContainer();
        left = new KrakenX60Motor(
                rootLogger.name("left"),
                currentLog,
                new CanId(6),
                NeutralMode100.BRAKE,
                MotorPhase.FORWARD,
                new CurrentLimit(50, 50),
                new Friction(0, 0, 0, 0),
                PIDConstants.makeVelocityPID(0.03));
        right = new KrakenX60Motor(
                rootLogger.name("right"),
                currentLog,
                new CanId(7),
                NeutralMode100.BRAKE,
                MotorPhase.REVERSE,
                new CurrentLimit(50, 50),
                new Friction(0, 0, 0, 0),
                PIDConstants.makeVelocityPID(0.03));
    }

    @Override
    public void robotPeriodic() {
         // Advance the drumbeat.
        Takt.update();
        // Take all the measurements we can, as soon and quickly as possible.
        Cache.refresh();
     
        CommandScheduler.getInstance().run();
        left.periodic();
        right.periodic();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        left.setVelocity(250, 0);// left.setDutyCycle(1.0);
        right.setVelocity(250, 0);// right.setDutyCycle(1.0);
    }

    @Override
    public void teleopPeriodic() {
        left.setDutyCycle(0.5);
        right.setDutyCycle(0.5);
        // System.out.println("Running!");
    }

    @Override
    public void teleopExit() {
    }
}
