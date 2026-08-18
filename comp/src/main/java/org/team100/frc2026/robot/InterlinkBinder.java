package org.team100.frc2026.robot;

import static org.team100.lib.util.TriggerUtil.onTrue;
import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.controller.r1.AzimuthController;
import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.hid.InterLinkDX;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.subsystems.swerve.commands.manual.DriveFieldRelative;
import org.team100.lib.subsystems.swerve.commands.manual.DriveMovingTargetLock;

import edu.wpi.first.wpilibj.RobotState;

/**
 * Control bindings for the Interlink DX. Also default commands.
 * 
 * See https://my.spektrumrc.com/ProdInfo/Files/SPMRFTX1-Manual-EN.pdf
 */
public class InterlinkBinder {

    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    @SuppressWarnings("unused")
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;
    private final Machinery m_machinery;
    final LoggerFactory m_log;

    public InterlinkBinder(Machinery machinery) {
        m_machinery = machinery;
        m_log = rootLogger.name("Commands");

        ////////////////////////////////////////////////////
        ///
        /// CONTROLLER
        ///
        InterLinkDX driver = new InterLinkDX(0);

        ////////////////////////////////////////////////////
        ///
        /// DEFAULT COMMANDS
        ///
        m_machinery.m_drive.setDefaultCommand(
                new DriveFieldRelative(
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        driver::velocity,
                        m_machinery.m_localizer::setHeedRadiusM,
                        m_machinery.m_drive,
                        m_machinery.m_limiter));
        m_machinery.m_shooter.setDefaultCommand(
                m_machinery.m_shooter.stop());
        m_machinery.m_intake.setDefaultCommand(
                m_machinery.m_intake.stop());
        m_machinery.m_intakeExtend.setDefaultCommand(
                m_machinery.m_intakeExtend.stop());

        ////////////////////////////////////////////////////
        ///
        /// DISORIENT
        ///
        onTrue(driver::reset, m_machinery.disorient());

        ////////////////////////////////////////////////////
        ///
        /// INTAKE
        ///
        whileTrue(driver::c2,
                m_machinery.m_intakeExtend.goToRetractedPosition());
        whileTrue(driver::c0,
                m_machinery.m_intakeExtend.goToExtendedPosition()
                        .andThen(m_machinery.m_intake.intake()));

        ////////////////////////////////////////////////////
        ///
        /// AIM
        ///
        FeedbackR1 thetaFeedback = new PIDFeedback(
                m_log, 3.2, 0, 0, true, 0.05, 1);

        AzimuthController aim = new AzimuthController(
                m_log,
                m_machinery.m_swerveKinodynamics::getMaxAngleSpeedRad_S,
                thetaFeedback);
        whileTrue(() -> driver.a1(),
                new DriveMovingTargetLock(
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        aim,
                        driver::velocity,
                        m_machinery.m_localizer::setHeedRadiusM,
                        m_machinery.m_limiter,
                        m_machinery.m_cachedSolution,
                        m_machinery.m_drive)
                        .withName("Target lock"));

        ////////////////////////////////////////////////////
        ///
        /// SHOOT
        ///
        // whileTrue(driver::i,
        // parallel(
        // m_machinery.m_shooter.shooterFullspeed(),
        // Commands.repeatingSequence(
        // waitUntil(m_machinery.m_shooter::atSpeed)
        // .onlyWhile(m_machinery.m_shooter::atSpeed))));

        ////////////////////////////////////////////////////
        ///
        /// TEST
        ///
        Tester tester = new Tester(m_machinery);
        whileTrue(() -> (RobotState.isTest() && driver.reset() && driver.cancel()),
                tester.prematch());

    }

}
