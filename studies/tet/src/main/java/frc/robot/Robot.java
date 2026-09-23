// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dynamics.p.PDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.motor.rev.NeoVortexCANSparkMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private static final CanId CAN_ID_1 = new CanId(20);
    private static final CanId CAN_ID_2 = new CanId(16);
    private static final double TOLERANCE_M_S = 1;
    private static final double GEAR_RATIO = 30.0 / 12.0;
    private static final double WHEEL_DIAMETER_M = 0.05;
    private static final double NORMAL_SPEED = 10;
    PDynamics dynamics = PDynamics.drum(0.001, 0.025);
    final Motor m1;
    final Motor m2;

    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    private static final TotalCurrentLog currentLog = new TotalCurrentLog(rootLogger);

    public Robot() {
        m_robotContainer = new RobotContainer();
        LoggerFactory parent = rootLogger.type(this);
        LoggerFactory topLog = parent.name("Top");
        LoggerFactory bottomLog = parent.name("Bottom");
        Friction friction = new Friction(0.5, 0.5, 0.0, 0.5);
        // tuned 3/12/26
        PIDConstants pid = PIDConstants.makeVelocityPID(0.08);
        m1 = new KrakenX44Motor(
                topLog, currentLog, CAN_ID_1, NeutralMode100.COAST, MotorPhase.FORWARD,
                new CurrentLimit(1, 1), friction, pid);
        m2 = new KrakenX44Motor(
                bottomLog, currentLog, CAN_ID_2, NeutralMode100.COAST, MotorPhase.REVERSE,
                new CurrentLimit(1, 1), friction, pid);
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        m1.periodic();
        m2.periodic();
        CommandScheduler.getInstance().run();
        NetworkTableInstance.getDefault().flush();
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
        m1.setVelocity(Math.PI * 20, 0);
        m2.setVelocity(Math.PI * 20, 0);
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
