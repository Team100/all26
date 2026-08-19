// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    // private final NeoVortexCANSparkMotor top;
    // private final NeoVortexCANSparkMotor bottom;
    private final KrakenX60Motor left;
    // private final KrakenX60Motor right;

    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    private static final TotalCurrentLog currentLog = new TotalCurrentLog(rootLogger);

    public Robot() {
        m_robotContainer = new RobotContainer();
        // top = new NeoVortexCANSparkMotor(
        // rootLogger,
        // currentLog,
        // new CanId(1),
        // NeutralMode100.BRAKE,
        // MotorPhase.FORWARD,
        // new CurrentLimit(1, 1),
        // new Friction(0, 0, 0, 0),
        // PIDConstants.zero(), 0, 0);
        // bottom = new NeoVortexCANSparkMotor(
        // rootLogger,
        // currentLog,
        // new CanId(2),
        // NeutralMode100.BRAKE,
        // MotorPhase.FORWARD,
        // new CurrentLimit(1, 1),
        // new Friction(0, 0, 0, 0),
        // PIDConstants.makeVelocityPID(1), 0, 0);
        left = new KrakenX60Motor(rootLogger, currentLog, new CanId(6), NeutralMode100.BRAKE, MotorPhase.FORWARD,
                new CurrentLimit(1, 1), new Friction(0, 0, 0, 0), PIDConstants.zero());
    //     right = new KrakenX60Motor(rootLogger, currentLog, new CanId(7), NeutralMode100.BRAKE, MotorPhase.FORWARD,
    //             new CurrentLimit(1, 1), new Friction(0, 0, 0, 0), PIDConstants.zero());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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
        left.setDutyCycle(0.05);
        // bottom.setDutyCycle(1);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
