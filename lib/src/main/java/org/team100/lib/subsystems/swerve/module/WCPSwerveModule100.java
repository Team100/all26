package org.team100.lib.subsystems.swerve.module;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.sensor.position.absolute.CombinedRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.wpi.AS5048RotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.ctre.Talon6Encoder;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;

public class WCPSwerveModule100 extends SwerveModule100 {
    /**
     * WCP calls this "rotation ratio" here, we use the "flipped belt" which is the
     * fastest steering ratio.
     * 12t -> 24t
     * 14t -> 72t
     * = 72 / 7
     * https://docs.wcproducts.com/wcp-swervex/misc/other-configurations/ratio-options
     */
    private static final double STEERING_RATIO = 10.28571429;

    /**
     * Flipped belt ratios.
     * 
     * See
     * https://docs.wcproducts.com/wcp-swervex/misc/other-configurations/ratio-options
     */
    public enum DriveRatio {
        FAST(5.5),
        MEDIUM(6.55);

        private double m_ratio;

        DriveRatio(double ratio) {
            m_ratio = ratio;
        }
    }

    // WCP 4 inch wheel
    private static final double WHEEL_DIAMETER_M = 0.094; // 0.1015

    /**
     * MAKE SURE THAT THE BEVELS ON THE WHEELS FOR ZEROING GO TO THE RIGHT
     */
    public static WCPSwerveModule100 getKrakenDriveKrakenSteer(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit driveLimit,
            CurrentLimit steerLimit,
            CanId driveMotorCanId,
            DriveRatio ratio,
            CanId turningMotorCanId,
            RoboRioChannel turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            EncoderDrive encoderDrive,
            NeutralMode100 neutral,
            MotorPhase motorPhase) {
        LinearMechanism drive = driveKraken(
                parent.name("Drive"),
                currentLog,
                driveLimit,
                driveMotorCanId,
                ratio);
        RotaryMechanism steer = steerKraken(
                parent.name("Turning"),
                currentLog,
                steerLimit,
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                STEERING_RATIO,
                kinodynamics,
                encoderDrive,
                neutral,
                motorPhase);
        return new WCPSwerveModule100(parent, drive, steer, ratio);
    }

    /**
     * MAKE SURE THAT THE BEVELS ON THE WHEELS FOR ZEROING GO TO THE RIGHT
     */
    public static WCPSwerveModule100 getFalconDriveFalconSteer(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit driveLimit,
            CurrentLimit steerLimit,
            CanId driveMotorCanId,
            DriveRatio ratio,
            CanId turningMotorCanId,
            RoboRioChannel turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            EncoderDrive encoderDrive,
            NeutralMode100 neutral,
            MotorPhase motorPhase) {
        LinearMechanism drive = driveFalcon(
                parent.name("Drive"),
                currentLog,
                driveLimit,
                driveMotorCanId,
                ratio);
        RotaryMechanism steer = steerFalcon(
                parent.name("Turning"),
                currentLog,
                steerLimit,
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                STEERING_RATIO,
                encoderDrive,
                neutral,
                motorPhase);
        return new WCPSwerveModule100(parent, drive, steer, ratio);
    }

    /**
     * MAKE SURE THAT THE BEVELS ON THE WHEELS FOR ZEROING GO TO THE RIGHT
     */
    public static WCPSwerveModule100 getKrakenDriveFalconSteer(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit driveLimit,
            CurrentLimit steerLimit,
            CanId driveMotorCanId,
            DriveRatio ratio,
            CanId turningMotorCanId,
            RoboRioChannel turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            EncoderDrive encoderDrive,
            NeutralMode100 neutral,
            MotorPhase motorPhase) {
        LinearMechanism drive = driveKraken(
                parent.name("Drive"),
                currentLog,
                driveLimit,
                driveMotorCanId,
                ratio);
        RotaryMechanism steer = steerFalcon(
                parent.name("Turning"),
                currentLog,
                steerLimit,
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                STEERING_RATIO,
                encoderDrive,
                neutral,
                motorPhase);
        return new WCPSwerveModule100(parent, drive, steer, ratio);
    }

    private static LinearMechanism driveKraken(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId driveMotorCanId,
            DriveRatio ratio) {
        // note (10/2/24) 0.4 produces oscillation, on carpet.
        Friction friction = new Friction(0.26, 0.26, 0.006, 0.5);
        // 3/14/26 lowered P from 0.05 to 0.03 to investigate oscillation
        PIDConstants pid = PIDConstants.makeVelocityPID(0.03);
        KrakenX60Motor driveMotor = new KrakenX60Motor(
                parent,
                currentLog,
                driveMotorCanId,
                NeutralMode100.COAST,
                MotorPhase.FORWARD,
                limit,
                friction,
                pid);
        Talon6Encoder encoder = driveMotor.encoder();
        return new LinearMechanism(parent,
                driveMotor,
                encoder,
                ratio.m_ratio,
                WHEEL_DIAMETER_M,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
    }

    private static LinearMechanism driveFalcon(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId driveMotorCanId,
            DriveRatio ratio) {
        Friction friction = new Friction(0.260, 0.260, 0.002, 0.5);
        PIDConstants pid = PIDConstants.makeVelocityPID(0.05);
        Falcon500Motor driveMotor = new Falcon500Motor(
                parent,
                currentLog,
                driveMotorCanId,
                NeutralMode100.COAST,
                MotorPhase.FORWARD,
                limit,
                friction,
                pid);
        Talon6Encoder encoder = driveMotor.encoder();
        return new LinearMechanism(parent,
                driveMotor, encoder, ratio.m_ratio, WHEEL_DIAMETER_M, Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
    }

    private static RotaryMechanism steerFalcon(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId turningMotorCanId,
            RoboRioChannel turningEncoderChannel,
            double turningOffset,
            double gearRatio,
            EncoderDrive drive,
            NeutralMode100 neutral,
            MotorPhase motorPhase) {
        Friction friction = new Friction(0.100, 0.100, 0.005, 0.5);

        // Talon outboard POSITION PID
        // 10/2/24 drive torque produces about a 0.5 degree deviation so maybe
        // this is too low.
        PIDConstants pid = PIDConstants.makePositionPID(2.0);

        Falcon500Motor turningMotor = new Falcon500Motor(
                parent,
                currentLog,
                turningMotorCanId,
                neutral,
                motorPhase,
                limit,
                friction,
                pid);

        // this reads the steering angle directly.
        RotaryPositionSensor turningSensor = new AS5048RotaryPositionSensor(
                parent,
                turningEncoderChannel,
                turningOffset,
                drive);

        Talon6Encoder builtInEncoder = turningMotor.encoder();

        ProxyRotaryPositionSensor proxy = new ProxyRotaryPositionSensor(builtInEncoder, gearRatio);
        CombinedRotaryPositionSensor combined = new CombinedRotaryPositionSensor(parent, turningSensor, proxy);

        return new RotaryMechanism(     
                parent, turningMotor, combined, gearRatio,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    private static RotaryMechanism steerKraken(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId turningMotorCanId,
            RoboRioChannel turningEncoderChannel,
            double turningOffset,
            double gearRatio,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            NeutralMode100 neutral,
            MotorPhase motorPhase) {
        Friction friction = new Friction(0.100, 0.100, 0.005, 0.5);

        // Talon outboard POSITION PID
        // 10/2/24 drive torque produces about a 0.5 degree deviation so maybe
        // this is too low.
        PIDConstants pid = PIDConstants.makePositionPID(1.0);

        KrakenX60Motor turningMotor = new KrakenX60Motor(
                parent,
                currentLog,
                turningMotorCanId,
                neutral,
                motorPhase,
                limit,
                friction,
                pid);

        // this reads the steering angle directly.
        RotaryPositionSensor turningSensor = new AS5048RotaryPositionSensor(
                parent,
                turningEncoderChannel,
                turningOffset,
                drive);

        Talon6Encoder builtInEncoder = turningMotor.encoder();

        ProxyRotaryPositionSensor proxy = new ProxyRotaryPositionSensor(builtInEncoder, gearRatio);
        CombinedRotaryPositionSensor combined = new CombinedRotaryPositionSensor(parent, turningSensor, proxy);

        return new RotaryMechanism(
                parent, turningMotor, combined, gearRatio,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    private WCPSwerveModule100(
            LoggerFactory log,
            LinearMechanism drive,
            RotaryMechanism steer,
            DriveRatio ratio) {
        // primary is 2:1 so final is whatever is left.
        super(log, drive, steer, WHEEL_DIAMETER_M, ratio.m_ratio / 2);
    }
}
