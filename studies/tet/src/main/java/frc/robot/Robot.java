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
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.servo.LinearPositionServo;
import org.team100.lib.servo.OutboardLinearPositionServo;
import org.team100.lib.state.StateR1;
import org.team100.lib.util.CanId;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private static final CanId LeftCan = new CanId(10);
    private static final CanId RightCan = new CanId(9);
    PDynamics dynamics = PDynamics.drum(0.001, 0.025);
    final Motor m1;
    final Motor m2;
    private final LinearMechanism l1;
    private final LinearMechanism l2;
    private final LinearPositionServo s1;
    private final LinearPositionServo s2;
    private final ReferenceR1 ref;
    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    private static final TotalCurrentLog currentLog = new TotalCurrentLog(rootLogger);
    private final XboxController controller;

    public Robot() {
        LoggerFactory parent = rootLogger.type(this);
        LoggerFactory leftLog = parent.name("left");
        LoggerFactory rightLog = parent.name("right");
        controller = new XboxController(0);
        Friction friction = new Friction(0.5, 0.5, 0.0, 0.5);
        // tuned 3/12/26
        PIDConstants pid = PIDConstants.makeVelocityPID(0.0275);
        PDynamics pd = new PDynamics(0);
        ProfileR1 profile = new TrapezoidProfileR1(0.5, 1, 0.01);
        ref = new ProfileReferenceR1(
                parent, () -> profile, 0.1, 10.1);
        m1 = new KrakenX44Motor(
                leftLog, currentLog, LeftCan, NeutralMode100.COAST, MotorPhase.REVERSE,
                new CurrentLimit(40, 50),
                friction, pid);
        m2 = new KrakenX44Motor(
                rightLog, currentLog, RightCan, NeutralMode100.COAST, MotorPhase.FORWARD,
                new CurrentLimit(40, 50),
                friction, pid);
        l1 = new LinearMechanism(
                leftLog, m1, m1.encoder(), 6.06,
                0.025, 0, 0.1);
        l2 = new LinearMechanism(
                rightLog, m2, m2.encoder(), 6.06,
                0.025, 0, 0.1);
        s1 = new OutboardLinearPositionServo(
                leftLog, l1, pd, ref, 0.01, 0.01);
        s2 = new OutboardLinearPositionServo(
                rightLog, l2, pd, ref, 0.01, 0.01);
        ref.setGoal(new StateR1(l1.getPositionM(), 0));

    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        s1.periodic();
        s2.periodic();
        CommandScheduler.getInstance().run();
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void teleopInit() {
        // m1.setVelocity(Math.PI * 200, 0);
        // m2.setVelocity(Math.PI * 200, 0);
        // m1.setVelocity(2, 0);
        // m2.setVelocity(2, 0);
    }

    @Override
    public void teleopPeriodic() {
        if (controller.getXButton()) {
            s1.setPositionProfiled(0.1);
            s2.setPositionProfiled(0.1);
        } else if (controller.getYButton()) {
            s1.setPositionProfiled(0);
            s2.setPositionProfiled(0);
        } else if (controller.getAButton()) {
            l1.setZero();
            l2.setZero();
        } else {
            m1.stop();
            m2.stop();
            ref.init(new StateR1(l1.getPositionM(), 0));
            s1.reset();
            s2.reset();
        }
    }

    @Override
    public void teleopExit() {
    }

}
