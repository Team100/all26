package org.team100.frc2026.auton;

import java.util.function.Consumer;

import org.team100.frc2026.robot.Machinery;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.config.AutonChooser;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Populates the Auton chooser with all available autons.
 * 
 * It's a good idea to instantiate them all here, even if you're not using them
 * all, even if they're just in development, so they don't rot.
 * 
 * In 2026, the field is rotationally symmetric, so there's no need to fill out
 * the "Alliance" part of the annotated command.
 */
public class Autons {
    private final AutonChooser m_autonChooser;

    public Autons(Machinery machinery) {
        ControllerSE2 controller = machinery.m_holonomicController;
        m_autonChooser = new AutonChooser();
        LoggerFactory log = Logging.instance().rootLogger.name("Auton");
        m_autonChooser.add(new JustShoot(machinery));
        m_autonChooser.add(new DoNothing());

        // These are for training, see
        // https://docs.google.com/document/d/1sSncYjKAp9Fh6xd_zrAt39hOhBcCyGkcvSjClX0NMxk
        m_autonChooser.add(new AheadTwoMeters(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new AheadTwoMetersWithRotation(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new UTurnWithRotation(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));

        // The following were from CVE in 2026
        m_autonChooser.add(new ScallopLobAuton(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new ScallopLobAutonRight(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new ScallopLobAutonLeft(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new TrenchScallopLobAutonRight(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new TrenchScallopLobAutonRightFull(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new TrenchScallopLobAutonLeft(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new TrenchScallopLobAutonLeftFull(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new MajorDisruptLTrench(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new MajorDisruptLBump(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new MajorDisruptRBump(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new MajorDisruptRTrench(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new MiddleDefenseLBump(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new MiddleDefenseRTrench(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new MiddleDefenseLTrench(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new MajorDefenseLTrench(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
        m_autonChooser.add(new MajorDefenseRTrench(
                log,
                machinery.m_swerveKinodynamics,
                controller,
                machinery));
    }

    public Command get() {
        AnnotatedCommand annotatedCommand = getAnnotated();
        if (annotatedCommand == null)
            return null;
        return annotatedCommand.command();
    }

    public AnnotatedCommand getAnnotated() {
        return m_autonChooser.get();
    }

    public void onChange(Consumer<AnnotatedCommand> listener) {
        m_autonChooser.onChange(listener);
    }

    public void close() {
        m_autonChooser.close();
    }

}
