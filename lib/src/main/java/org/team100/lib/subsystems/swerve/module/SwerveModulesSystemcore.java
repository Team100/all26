package org.team100.lib.subsystems.swerve.module;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.subsystems.swerve.module.WCPSwerveModule100.DriveRatio;
import org.team100.lib.util.CanId;

/**
 * For the new comp swerve drive that uses the systemcore alpha.
 */
public class SwerveModulesSystemcore extends SwerveModuleCollection {
    public SwerveModulesSystemcore(
            LoggerFactory log,
            TotalCurrentLog currentLog,
            CurrentLimit driveLimit,
            CurrentLimit steerLimit) {
        super(
                WCPSwerveModule100.getKrakenDriveKrakenSteerRedux(
                        log.name("Front Left"), currentLog, driveLimit, steerLimit,
                        new CanId(1), // drive
                        DriveRatio.MEDIUM,
                        new CanId(2), // steer
                        new CanId(1), // encoder
                        -0.993286,
                        EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                WCPSwerveModule100.getKrakenDriveKrakenSteerRedux(
                        log.name("Front Right"), currentLog, driveLimit, steerLimit,
                        new CanId(3), // drive
                        DriveRatio.MEDIUM,
                        new CanId(4), // steer
                        new CanId(2), // encoder
                        0.000427,
                        EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                WCPSwerveModule100.getKrakenDriveKrakenSteerRedux(
                        log.name("Rear Left"), currentLog, driveLimit, steerLimit,
                        new CanId(5), // drive
                        DriveRatio.MEDIUM,
                        new CanId(6), // steer
                        new CanId(3), // encoder
                        -0.003967,
                        EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                WCPSwerveModule100.getKrakenDriveKrakenSteerRedux(
                        log.name("Rear Right"), currentLog, driveLimit, steerLimit,
                        new CanId(7), // drive
                        DriveRatio.MEDIUM,
                        new CanId(8), // steer
                        new CanId(4), // encoder
                        -0.001465,
                        EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE));
        System.out.println("************** Kraken Drive, Kraken Steer, Redux Encoders **************");

    }
}
