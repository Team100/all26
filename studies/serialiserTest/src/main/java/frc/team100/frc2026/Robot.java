// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team100.frc2026;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    // private final m_robotContainer;
    // private final TalonFX first;
    private final TalonFX second;
    private final SparkFlex flex;
    private final SparkFlex flexs;

    public Robot() {
        // first = new TalonFX(9, new CANBus());
        flex = new SparkFlex(0, 14, MotorType.kBrushless);
        flexs = new SparkFlex(0, 1, MotorType.kBrushless);
        second = new TalonFX(4, new CANBus());
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
        // m_autonomousCommand = %m_robotContainer.getAutonomousCommand();

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
        // first.setThrottle(-1);
        second.setThrottle(-0.2);
        flex.setThrottle(0.2);
        flexs.setThrottle(0.2);

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {
    }
}
