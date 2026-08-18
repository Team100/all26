package org.team100.lib.examples.semiauto;

import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is an example of what you'd put in RobotContainer to use some of the
 * subsystem and command classes here.
 */
public class PositionAwareSubsystemSetup {
    public PositionAwareSubsystemSetup(
            LoggerFactory log,
            DriverXboxControl control,
            SwerveDriveSubsystem drive) {

        PositionAwareSubsytem shooterElevation = new PositionAwareSubsytem(
                new Translation2d(3, 4), drive::getPose);

        // you could run the elevation command as the default ...
        shooterElevation.setDefaultCommand(shooterElevation.holdAimingPoint());

        // ... or you could run it on demand:
        new Trigger(control::a).whileTrue(shooterElevation.holdAimingPoint());
    }
}
