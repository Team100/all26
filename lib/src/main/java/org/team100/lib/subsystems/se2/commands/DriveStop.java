package org.team100.lib.subsystems.se2.commands;

import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;

import org.wpilib.command2.Command;

/**
 * Stop the drivetrain.
 */
public class DriveStop extends Command {
    private final VelocitySubsystemSE2 m_drive;

    public DriveStop(VelocitySubsystemSE2 robotDrive) {
        m_drive = robotDrive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.stop();
    }
}
