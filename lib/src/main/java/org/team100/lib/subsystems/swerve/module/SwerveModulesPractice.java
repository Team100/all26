package org.team100.lib.subsystems.swerve.module;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.subsystems.swerve.module.WCPSwerveModule100.DriveRatio;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;

/**
 * For the practice swerve drive.
 */
public class SwerveModulesPractice extends SwerveModuleCollection {
    public SwerveModulesPractice(
            LoggerFactory log,
            TotalCurrentLog currentLog,
            CurrentLimit driveLimit,
            CurrentLimit steerLimit) {
        super(
                WCPSwerveModule100.getFalconDriveFalconSteer(
                        log.name("Front Left"), currentLog, driveLimit, steerLimit,
                        new CanId(12), // drive
                        DriveRatio.FAST,
                        new CanId(32), // steer
                        new RoboRioChannel(6),
                        0.648451,
                        EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                WCPSwerveModule100.getFalconDriveFalconSteer(
                        log.name("Front Right"), currentLog, driveLimit, steerLimit,
                        new CanId(11), // drive
                        DriveRatio.FAST,
                        new CanId(30), // steer
                        new RoboRioChannel(8),
                        0.875511,
                        EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                WCPSwerveModule100.getFalconDriveFalconSteer(
                        log.name("Rear Left"), currentLog, driveLimit, steerLimit,
                        new CanId(21), // drive
                        DriveRatio.FAST,
                        new CanId(31), // steer
                        new RoboRioChannel(7),
                        0.409354,
                        EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                WCPSwerveModule100.getFalconDriveFalconSteer(
                        log.name("Rear Right"), currentLog, driveLimit, steerLimit,
                        new CanId(22), // drive
                        DriveRatio.FAST,
                        new CanId(33), // steer
                        new RoboRioChannel(9),
                        0.029534,
                        EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE));
        System.out.println("************** Falcon Drive, Falcon Steer, Duty-Cycle Encoders **************");
    }
}
