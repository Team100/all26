package org.team100.ballerina;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.frc2026.targeting.Targeter;
import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.indicator.SolidIndicator;
import org.team100.lib.localization.ManualPose;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.turret.Turret;
import org.team100.lib.util.Banner;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;
import org.team100.lib.visualization.Ball;
import org.team100.lib.visualization.BallFactory;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.button.Trigger;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.util.Color;

public class Robot extends TimedRobot {
    private static final boolean BALL_2D = false;
    private final DriverXboxControl m_controller;
    private final Turret m_turret;
    private final ManualPose m_pose;
    private final SolidIndicator m_indicator;
    private final Ball m_ball;

    public Robot() {
        Banner.printBanner();
        Experiments.instance.show();
        Logging log = Logging.instance();
        LoggerFactory fieldLogger = log.fieldLogger;
        LoggerFactory rootLogger = log.rootLogger;
        TotalCurrentLog currentLog = new TotalCurrentLog(rootLogger);
        m_controller = new DriverXboxControl(rootLogger, 0);
        m_pose = new ManualPose(fieldLogger, m_controller::velocity, new Pose2d(6, 4, Rotation2d.kZero));
        // use the 2026 target selector
        Supplier<Optional<Translation2d>> target = () -> {
            return FieldConstants2026.TARGET(
                    m_pose.getState().translation());
        };
        // use the 2026 targeting table
        Targeter targeter = new Targeter(() -> m_pose.getState().translation());
        m_turret = new Turret(
                rootLogger,
                currentLog,
                fieldLogger,
                new CanId(0),
                new CanId(0),
                new CanId(0),
                targeter::forRange,
                m_pose::getState,
                target);
        m_indicator = new SolidIndicator(new RoboRioChannel(0), 40);
        m_indicator.state(this::indicatorState);
        if (BALL_2D) {
            m_ball = BallFactory.get2d(
                    fieldLogger, m_pose::getState, m_turret::getAzimuth, m_turret::getSpeed);
        } else {
            m_ball = BallFactory.get3d(
                    rootLogger,
                    fieldLogger,
                    m_pose::getState,
                    m_turret::getAzimuth,
                    m_turret::getElevation,
                    m_turret::getSpeed,
                    m_turret::getOmega);
        }
        // button 1
        new Trigger(m_controller::a).whileTrue(m_turret.aim());
        // button 2
        new Trigger(m_controller::b).whileTrue(m_ball.shoot());

        m_turret.setDefaultCommand(m_turret.stop());
    }

    private Color indicatorState() {
        if (m_turret.aiming()) {
            if (m_turret.solved()) {
                if (m_turret.onTarget()) {
                    // ready to fire
                    return Color.GREEN;
                } else {
                    if (m_turret.validSetpoint()) {
                        // solved, valid, but not on target yet
                        return Color.YELLOW;
                    } else {
                        // setpoint is inaccessible
                        return Color.ORANGE;
                    }
                }
            } else {
                // no solution
                return Color.RED;
            }
        } else {
            // idle
            return Color.LAVENDER;
        }
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_pose.periodic();
        m_indicator.periodic();
        m_ball.periodic();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void close() {
        m_indicator.close();
    }
}
