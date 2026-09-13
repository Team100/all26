package org.team100.rrr.robot;

import static org.team100.lib.util.TriggerUtil.onTrue;
import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.profile.r1.WPITrapezoidProfileR1;
import org.team100.lib.subsystems.rrr.RRRProfile;
import org.team100.lib.subsystems.rrr.commands.MoveJointsManually;
import org.team100.lib.subsystems.rrr.commands.MoveManuallySE2;
import org.team100.lib.subsystems.rrr.commands.MoveToConfigWithProfile;
import org.team100.lib.subsystems.rrr.commands.MoveWithProfile;
import org.team100.lib.subsystems.rrr.commands.MoveWithSpline;
import org.team100.lib.subsystems.rrr.commands.MoveWithTrajectorySE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Binder {
    private final Machinery m_machinery;
    private final DriverXboxControl m_driver;

    public Binder(LoggerFactory rootLogger, Machinery machinery) {
        LoggerFactory log = rootLogger.type(this);
        m_machinery = machinery;
        m_driver = new DriverXboxControl(log, 0);

        // Default control is "joint" mode.
        // m_machinery.m_arm.setDefaultCommand(
        // m_machinery.m_arm.moveJointsManually(m_driver::velocity));
        ProfileR1 p = new WPITrapezoidProfileR1(1, 2);
        RRRProfile prof = new RRRProfile(
                m_machinery.m_arm.kinematics(), m_machinery.m_arm.feasibility(), p);
        MoveManuallySE2 manual = new MoveManuallySE2(m_machinery.m_arm, m_driver::velocity, prof);
        Command joints = new MoveJointsManually(m_machinery.m_arm, m_driver::velocity);

        // default stops the arm
        m_machinery.m_arm.setDefaultCommand(m_machinery.m_arm.run(m_machinery.m_arm::stop));

        // Left bumper (button 5, "b" in sim) is "pose" mode
        whileTrue(m_driver::leftBumper, Commands.runOnce(
                () -> m_machinery.m_viz.setT(manual::getT))
                .andThen(manual));
        // Right bumper (button 6, "n" in sim) is "joint" mode.
        whileTrue(m_driver::rightBumper, Commands.runOnce(
                () -> m_machinery.m_viz.setT(null))
                .andThen(joints));

        // ProfileR1 profile = new WPITrapezoidProfileR1(3, 6);
        ProfileR1 profile = new TrapezoidProfileR1(3, 6, 0.01);

        // Force the arm back to the home position without kinematics, to escape a
        // kinematic trap. (button 8, comma in sim)
        whileTrue(m_driver::start,
                new MoveToConfigWithProfile(
                        m_machinery.m_arm, profile, new RRRConfig(0, 0, 0)));

        // Force the encoder values to zero.  Move the apparatus to
        // fully extended before running this, if you want the zero
        // to be in the right place.  :-) (button 7, "m" in sim)
        onTrue(m_driver::back,
                m_machinery.m_arm.run(m_machinery.m_arm::setZero));

        // profiles in joint space make kinda circular paths in workspace
        MoveAndHold move1 = new MoveWithProfile(
                m_machinery.m_arm, profile,
                new Pose2d(0.5, 0.25, new Rotation2d(0)));
        MoveAndHold move2 = new MoveWithProfile(
                m_machinery.m_arm, profile,
                new Pose2d(0.6, 0.25, new Rotation2d(0)));
        MoveAndHold move3 = new MoveWithProfile(
                m_machinery.m_arm, profile,
                new Pose2d(0.6, -0.25, new Rotation2d(0)));
        MoveAndHold move4 = new MoveWithProfile(
                m_machinery.m_arm, profile,
                new Pose2d(0.5, -0.25, new Rotation2d(0)));
        whileTrue(m_driver::b, // button 2
                move1.until(move1::isDone)
                        .andThen(move2.until(move2::isDone))
                        .andThen(move3.until(move3::isDone))
                        .andThen(move4.until(move4::isDone)));

        // spline ends are kinda straighter
        // note approach directions.
        MoveAndHold move1s = new MoveWithSpline(
                m_machinery.m_arm,
                new VelocitySE2(0, 0.3, 0),
                new Pose2d(0.45, 0.25, new Rotation2d(0)),
                new VelocitySE2(0, 0.3, 0));
        MoveAndHold move2s = new MoveWithSpline(
                m_machinery.m_arm,
                new VelocitySE2(0.15, 0, 0),
                new Pose2d(0.6, 0.25, new Rotation2d(0)),
                new VelocitySE2(0.15, 0, 0));
        MoveAndHold move3s = new MoveWithSpline(
                m_machinery.m_arm,
                new VelocitySE2(0, -0.3, 0),
                new Pose2d(0.6, -0.25, new Rotation2d(0)),
                new VelocitySE2(0, -0.3, 0));
        MoveAndHold move4s = new MoveWithSpline(
                m_machinery.m_arm,
                new VelocitySE2(-0.15, 0, 0),
                new Pose2d(0.45, -0.25, new Rotation2d(0)),
                new VelocitySE2(-0.15, 0, 0));
        whileTrue(m_driver::x,
                move1s.until(move1s::isDone)
                        .andThen(move2s.until(move2s::isDone))
                        .andThen(move3s.until(move3s::isDone))
                        .andThen(move4s.until(move4s::isDone)));

        // trajectories in workspace make straight lines in workspace
        MoveAndHold move1t = new MoveWithTrajectorySE2(
                log, m_machinery.m_arm,
                new Pose2d(0.45, 0.25, new Rotation2d(0)), 1);
        MoveAndHold move2t = new MoveWithTrajectorySE2(
                log, m_machinery.m_arm,
                new Pose2d(0.6, 0.25, new Rotation2d(0)), 1);
        MoveAndHold move3t = new MoveWithTrajectorySE2(
                log, m_machinery.m_arm,
                new Pose2d(0.6, -0.25, new Rotation2d(0)), 1);
        MoveAndHold move4t = new MoveWithTrajectorySE2(
                log, m_machinery.m_arm,
                new Pose2d(0.45, -0.25, new Rotation2d(0)), 1);
        whileTrue(m_driver::y,
                move1t.until(move1t::isDone)
                        .andThen(move2t.until(move2t::isDone))
                        .andThen(move3t.until(move3t::isDone))
                        .andThen(move4t.until(move4t::isDone)));
    }

    public void close() {
    }
}
