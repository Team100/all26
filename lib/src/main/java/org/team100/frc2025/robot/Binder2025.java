package org.team100.frc2025.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.BooleanSupplier;

import org.team100.frc2025.Climber.ClimberCommands;
import org.team100.frc2025.CommandGroups.MoveToAlgaePosition;
import org.team100.frc2025.Swerve.Auto.BigLoop;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.hid.Buttons2025;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.hid.OperatorXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.profile.se2.HolonomicProfileFactory;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.prr.commands.FollowJointProfiles;
import org.team100.lib.subsystems.se2.commands.DriveWithTrajectoryFunction;
import org.team100.lib.subsystems.se2.commands.FloorPickSequence2;
import org.team100.lib.subsystems.se2.commands.ManualPosition;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Binds buttons to commands. Also creates default commands.
 */
public class Binder2025 {
    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;
    private final Machinery2025 m_machinery;

    public Binder2025(Machinery2025 machinery) {
        m_machinery = machinery;
    }

    public void bind() {
        final LoggerFactory log = rootLogger.name("Commands");

        /////////////////////////////////////////////////
        ///
        /// CONTROLS
        ///
        final DriverXboxControl driver = new DriverXboxControl(0);
        final OperatorXboxControl operator = new OperatorXboxControl(1);
        final Buttons2025 buttons = new Buttons2025(2);

        /////////////////////////////////////////////////
        //
        // DEFAULT COMMANDS
        //
        // final FeedbackR1 thetaFeedback = new PIDFeedback(
        // log, 3.2, 0, 0, true, 0.05, 1);

        SwerveLimiter limiter = new SwerveLimiter(
                log,
                m_machinery.m_swerveKinodynamics,
                RobotController::getBatteryVoltage);
        limiter.updateSetpoint(new VelocityControlSE2(
                m_machinery.m_drive.getVelocity()));

        // There are 3 modes:
        // * normal
        // * lock rotation to reef center
        // * barge-assist (slow when near barge)
        // final Command driveDefault = new DriveManuallySimple(
        // driver::velocity,
        // m_machinery.m_localizer::setHeedRadiusM,
        // m_machinery.m_drive,
        // limiter,
        // new ManualWithProfiledReefLock(
        // log, m_machinery.m_swerveKinodynamics, driver::leftTrigger,
        // thetaFeedback, () -> m_machinery.m_drive.getPose().getTranslation()),
        // new ManualWithBargeAssist(
        // log, m_machinery.m_swerveKinodynamics, driver::pov,
        // thetaFeedback, m_machinery.m_drive::getPose),
        // driver::leftBumper);
        // m_machinery.m_drive.setDefaultCommand(driveDefault.withName("drive
        // default"));
        // // WARNING! This default command *MOVES IMMEDIATELY WHEN ENABLED*!
        m_machinery.m_mech.setDefaultCommand(m_machinery.m_mech.profileHomeAndThenRest().withName("mech default"));
        m_machinery.m_climber.setDefaultCommand(m_machinery.m_climber.stop().withName("climber default"));
        m_machinery.m_climberIntake
                .setDefaultCommand(m_machinery.m_climberIntake.stop().withName("climber intake default"));
        m_machinery.m_manipulator.setDefaultCommand(m_machinery.m_manipulator.stop().withName("manipulator default"));

        ///////////////////////////
        //
        // DRIVETRAIN
        //
        // Reset pose estimator so the current gyro rotation corresponds to zero.
        // onTrue(driver::back,
        // new SetRotation(m_machinery.m_drive, Rotation2d.kZero));

        // // Reset pose estimator so the current gyro rotation corresponds to 180.
        // onTrue(driver::start,
        // new SetRotation(m_machinery.m_drive, Rotation2d.kPi));

        ////////////////////////////////////////////////////////////
        //
        // MECHANISM
        //
        // "fly" the joints manually
        whileTrue(operator::leftBumper,
                new ManualPosition(operator::velocity, m_machinery.m_mech));
        // new ManualConfig(operatorControl::velocity, mech));

        ////////////////////////////////////////////////////////////
        //
        // CORAL PICK
        //

        // At the same time, move the arm to the floor and spin the intake,
        // and go back home when the button is released, ending when complete.
        whileTrue(driver::rightTrigger,
                parallel(
                        m_machinery.m_mech.pickWithProfile(),
                        m_machinery.m_manipulator.centerIntake()))
                .onFalse(m_machinery.m_mech.profileHomeTerminal());

        // Move to coral ground pick location.
        whileTrue(driver::rightBumper,
                parallel(
                        m_machinery.m_mech.pickWithProfile(),
                        m_machinery.m_manipulator.centerIntake()))
                .onFalse(m_machinery.m_mech.profileHomeTerminal());

        // go full speed for rotation, so that rotation gets there first.
        final ProfileSE2 coralPickProfile = HolonomicProfileFactory.freeRotationCurrentLimitedExponential(
                m_machinery.m_swerveKinodynamics, 0.5, 1.0);

        // Pick a game piece from the floor, based on camera input.

        // This is the old one, commented out while I work on the new one.
        // whileTrue(driver::x,
        // parallel(
        // m_machinery.m_mech.pickWithProfile(),
        // m_machinery.m_manipulator.centerIntake(),
        // FloorPickSequence.get(
        // log, fieldLogger, m_machinery.m_drive, m_machinery.m_targets,
        // ControllerFactorySE2.pick(log), coralPickProfile)
        // .withName("Floor Pick"))
        // .until(m_machinery.m_manipulator::hasCoral));

        // this skips the intaking, just to work on the drivetrain.
        whileTrue(driver::x,
                FloorPickSequence2.get(
                        log,
                        fieldLogger,
                        m_machinery.m_drive,
                        m_machinery.m_targets,
                        ControllerFactorySE2.pick(log),
                        coralPickProfile)
                        .withName("FloorPick"));

        // Sideways intake for L1
        whileTrue(buttons::red2,
                sequence(
                        m_machinery.m_manipulator.sidewaysIntake()
                                .until(m_machinery.m_manipulator::hasCoralSideways),
                        m_machinery.m_manipulator.sidewaysHold()));

        ////////////////////////////////////////////////////////////
        //
        // CORAL SCORING
        //
        // Manual movement of arm, for testing.
        whileTrue(buttons::l1, m_machinery.m_mech.profileHomeToL1());
        // whileTrue(buttons::l2, mech.homeToL2()).onFalse(mech.l2ToHome());
        // whileTrue(buttons::l3, mech.homeToL3()).onFalse(mech.l3ToHome());
        // whileTrue(buttons::l4, mech.homeToL4()).onFalse(mech.l4ToHome());
        // whileTrue(driverControl::test, m_mech.homeToL4()).onFalse(m_mech.l4ToHome());

        final LoggerFactory coralSequence = rootLogger.name("Coral Sequence");
        // final ProfileSE2 profile = HolonomicProfileFactory.get(
        // coralSequence, m_machinery.m_swerveKinodynamics, 1, 0.5, 1, 0.2);
        final ControllerSE2 holonomicController = ControllerFactorySE2.byIdentity(coralSequence);

        // Drive to a scoring location at the reef and score.
        whileTrue(driver::b, m_machinery.m_manipulator.centerEject());
        // whileTrue(driver::a,
        // ScoreCoralSmartLuke.get(
        // coralSequence, m_machinery.m_mech, m_machinery.m_manipulator,
        // holonomicController, profile, m_machinery.m_drive,
        // m_machinery.m_localizer::setHeedRadiusM, buttons::level, buttons::point));
        // // ScoreCoralSmart.get(
        // coralSequence, m_machinery.m_mech, m_machinery.m_manipulator,
        // holonomicController, profile, m_machinery.m_drive,
        // m_machinery.m_localizer::setHeedRadiusM, buttons::level, buttons::point));

        ///////////////////////////////////////////////////
        //
        // for testing
        //
        BigLoop bigloop = new BigLoop(m_machinery.m_swerveKinodynamics);
        DriveWithTrajectoryFunction navigator = new DriveWithTrajectoryFunction(
                rootLogger, m_machinery.m_drive, holonomicController, m_machinery.m_trajectoryViz,
                bigloop);
        whileTrue(driver::a,
                navigator.until(navigator::isDone));
        //
        //
        ///////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////
        //
        // ALGAE
        //
        // Algae commands have two components: one button for manipulator,
        // one button for arm mechanism.

        // grab and hold algae, and then eject it when you let go of the button
        onTrue(buttons::algae,
                MoveToAlgaePosition.get(
                        m_machinery.m_mech, buttons::algaeLevel, buttons::algae));

        FollowJointProfiles homeGentle = m_machinery.m_mech.homeAlgae();
        whileTrue(driver::b, m_machinery.m_mech.algaePickGround()).onFalse(homeGentle.until(homeGentle::isDone));

        // Intake algae and puke it when you let go.
        whileTrue(buttons::barge,
                sequence(
                        m_machinery.m_manipulator.algaeIntake()
                                .until(m_machinery.m_manipulator::hasAlgae),
                        m_machinery.m_manipulator.algaeHold()) //
        ).onFalse(
                m_machinery.m_manipulator.algaeEject()
                        .withTimeout(0.5));

        // Move mech to processor
        whileTrue(buttons::red4,
                m_machinery.m_mech.processorWithProfile());

        // Move mech to barge
        whileTrue(buttons::red3,
                m_machinery.m_mech.homeToBarge()).onFalse(m_machinery.m_mech.bargeToHome());

        // whileTrue(driverControl::a, m_manipulator.run(m_manipulator::intakeCenter));
        // whileTrue(driverControl::b, m_manipulator.run(m_manipulator::ejectCenter));
        // whileTrue(driverControl::x, m_manipulator.run(m_manipulator::intakeCenter));

        ////////////////////////////////////////////////////////////
        //
        // CLIMB
        //
        // Extend, spin, wait for intake, and pull climber in and drive forward.
        whileTrue(buttons::red1,
                ClimberCommands.climbIntake(m_machinery.m_climber, m_machinery.m_climberIntake, m_machinery.m_mech));

        // Step 2, driver: Pull climber in and drive forward.
        onTrue(driver::y,
                ClimberCommands.climb(m_machinery.m_climber, m_machinery.m_drive, m_machinery.m_mech));

        // Between matches, operator: Reset the climber position.
        whileTrue(operator::rightBumper,
                m_machinery.m_climber.manual(operator::leftY));

        ////////////////////////////////////////////////////////////
        //
        // TEST
        //
        Tester2025 tester = new Tester2025(m_machinery);
        whileTrue(() -> (RobotState.isTest() && driver.a() && driver.b()),
                tester.prematch());
    }

    private static Trigger whileTrue(BooleanSupplier condition, Command command) {
        return new Trigger(condition).whileTrue(command);
    }

    private static Trigger onTrue(BooleanSupplier condition, Command command) {
        return new Trigger(condition).onTrue(command);
    }

}
