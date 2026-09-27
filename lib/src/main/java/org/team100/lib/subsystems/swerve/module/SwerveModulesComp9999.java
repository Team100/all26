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

public class SwerveModulesComp9999 extends SwerveModuleCollection {
    public SwerveModulesComp9999(
            LoggerFactory log,
            TotalCurrentLog currentLog,
            CurrentLimit driveLimit,
            CurrentLimit steerLimit) {
        super(
                WCPSwerveModule100.getKrakenDriveKrakenSteer(
                        log.name("Front Left"), currentLog, driveLimit, steerLimit,
                        new CanId(3), // drive
                        DriveRatio.MEDIUM,
                        new CanId(44), // steer
                        new RoboRioChannel(8),
                        0.228237,
                        EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                WCPSwerveModule100.getKrakenDriveKrakenSteer(
                        log.name("Front Right"), currentLog, driveLimit, steerLimit,
                        new CanId(8), // drive
                        DriveRatio.MEDIUM,
                        new CanId(7), // steer
                        new RoboRioChannel(6),
                        0.817243,
                        EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                WCPSwerveModule100.getKrakenDriveKrakenSteer(
                        log.name("Rear Left"), currentLog, driveLimit, steerLimit,
                        new CanId(2), // drive
                        DriveRatio.MEDIUM,
                        new CanId(50), // steer
                        new RoboRioChannel(7),
                        0.147507,
                        EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                WCPSwerveModule100.getKrakenDriveKrakenSteer(
                        log.name("Rear Right"), currentLog, driveLimit, steerLimit,
                        new CanId(4), // drive
                        DriveRatio.MEDIUM,
                        new CanId(62), // steer
                        new RoboRioChannel(0),
                        0.835573,
                        EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE));
        System.out.println("************** Kraken Drive, Kraken Steer, Duty-Cycle Encoders **************");
    }
}
