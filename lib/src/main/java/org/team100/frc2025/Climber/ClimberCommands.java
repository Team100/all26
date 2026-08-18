package org.team100.frc2025.Climber;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.Swerve.DriveForwardSlowly;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberCommands {
    /**
     * Set arm position and start spinning.
     */
    public static Command climbIntake(Climber2025 climber, ClimberIntake intake, CalgamesMech mech) {
        return parallel(
                climber.goToIntakePosition(),
                intake.intake(),
                mech.climbWithProfile())
                .withName("climb intake");
    }

    /**
     * Pull the climber in while driving slowly forward.
     */
    public static Command climb(Climber2025 climber, SwerveDriveSubsystem drive, CalgamesMech mech) {
        return parallel(
                climber.goToClimbPosition(),
                new DriveForwardSlowly(drive),
                mech.setDisabled(true) //
        ).withName("climb while driving");

    }

    /**
     * Pull the climber in while driving slowly forward.
     */
    public static Command climbAuto(Climber2025 climber, ClimberIntake intake, SwerveDriveSubsystem drive,
            CalgamesMech shoulder) {
        return sequence(
                climbIntake(climber, intake, shoulder), climb(climber, drive, shoulder));

    }
}
