package org.team100.lib.subsystems.swerve.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.function.UnaryOperator;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.FusedEstimator;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.SimulatedGyro;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.SwerveLocal;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.subsystems.swerve.module.SwerveModulesSim;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SimulatedDrivingTest implements Timeless {
    private static final boolean DEBUG = false;
    final LoggerFactory fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());

    final SwerveKinodynamics swerveKinodynamics;
    final SwerveModuleCollection collection;
    final Gyro gyro;
    final SwerveLocal swerveLocal;
    final SwerveLimiter limiter;
    final SwerveDriveSubsystem drive;

    SimulatedDrivingTest() throws IOException {
        LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
        swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        collection = new SwerveModulesSim(logger);
        gyro = new SimulatedGyro(logger, swerveKinodynamics, collection, 0);
        swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        UnaryOperator<Twist2d> odometryNoise = UnaryOperator.identity();

        FusedEstimator estimate = new FusedEstimator(
                logger, fieldLogger, swerveKinodynamics, odometryNoise, layout, gyro, swerveLocal);
        limiter = new SwerveLimiter(logger, swerveKinodynamics, () -> 12);

        drive = new SwerveDriveSubsystem(
                logger,
                estimate,
                swerveLocal);
    }

    @Test
    void testSteps() {
        VelocitySE2 input = new VelocitySE2(2, 0, 3.5);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds targetChassisSpeeds = SwerveKinodynamics.toInstantaneousChassisSpeeds(input, theta);
        SwerveModuleStates states = swerveKinodynamics.toSwerveModuleStates(targetChassisSpeeds);

        // mmmm the angles start as zero? does this matter? no?
        SwerveModulePositions startPositions = new SwerveModulePositions(
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100());

        // say each module proceeds at its setpoint speed and angle (i.e. starting angle
        // is irrelevant)

        double dt = 0.02;
        SwerveModulePositions endPositions = new SwerveModulePositions(
                new SwerveModulePosition100(
                        states.frontLeft().speed() * dt,
                        states.frontLeft().angle()),
                new SwerveModulePosition100(
                        states.frontRight().speed() * dt,
                        states.frontRight().angle()),
                new SwerveModulePosition100(
                        states.rearLeft().speed() * dt,
                        states.rearLeft().angle()),
                new SwerveModulePosition100(
                        states.rearRight().speed() * dt,
                        states.rearRight().angle()));

        SwerveModuleDeltas modulePositionDelta = SwerveModuleDeltas.modulePositionDelta(
                startPositions,
                endPositions);
        if (DEBUG)
            System.out.printf("%s\n", modulePositionDelta);

        Twist2d twist = swerveKinodynamics.getKinematics().forward(modulePositionDelta);

        Pose2d deltaPose = GeometryUtil.sexp(twist);
        ChassisSpeeds continuousSpeeds = new ChassisSpeeds(
                deltaPose.getX(),
                deltaPose.getY(),
                deltaPose.getRotation().getRadians()).div(dt);

        // to pass, this requires the "veering correction" to be zero.
        assertEquals(0, continuousSpeeds.vyMetersPerSecond, 1e-12);
    }

    @Test
    void testStraight() {
        // just +x
        VelocityControlSE2 input = new VelocityControlSE2(2, 0, 0);
        double start = Takt.get();
        for (int i = 0; i < 100; ++i) {
            stepTime();
            drive.set(input);
            if (DEBUG)
                System.out.printf("%.2f %s\n", Takt.get() - start, drive.getState().pose());
        }
    }

    @Test
    void testStraightVerbatim() {
        // just +x
        // this accelerates infinitely, immediately to the requested speed.
        VelocityControlSE2 input = new VelocityControlSE2(2, 0, 0);
        double start = Takt.get();
        for (int i = 0; i < 100; ++i) {
            stepTime();
            drive.set(input);
            if (DEBUG)
                System.out.printf("%.2f %s\n", Takt.get() - start, drive.getState().pose());
        }
    }

    /**
     * Uses the setpoint generator. turn on DEBUG in SwerveLocal to see the bug, the
     * setpoint generator output is not course-invariant.
     * 
     * accel is 10 m/s/s; dt is 0.02, so dv is 0.2.
     */
    @Test
    void testVeering() {
        Experiments.INSTANCE.override(Experiment.UseSwerveLimiter, true);
        // +x and spinning. course is always zero.
        VelocityControlSE2 input = new VelocityControlSE2(2, 0, 3.5);
        for (int i = 0; i < 50; ++i) {
            if (DEBUG)
                System.out.printf("\nstep time ...\n");
            stepTime();
            if (DEBUG)
                System.out.printf("takt: %.2f state: %s\n", Takt.get(), drive.getState());
            drive.set(input);
        }
    }

    /**
     * No veering. Drive commands go to simulated motors, which respond instantly.
     */
    @Test
    void testVeeringVerbatim() {
        // +x and spinning
        VelocityControlSE2 input = new VelocityControlSE2(2, 0, 3.5);
        for (int i = 0; i < 100; ++i) {
            if (DEBUG)
                System.out.printf("\nstep time ...\n");
            stepTime();
            if (DEBUG)
                System.out.printf("takt: %.2f state: %s\n", Takt.get(), drive.getState());
            drive.set(input);
        }
    }

    /** Is the gyro in sync with the estimated pose? Yes. */
    @Test
    void testGyro() {
        // spin fast
        VelocityControlSE2 input = new VelocityControlSE2(0, 0, 4);
        if (DEBUG)
            System.out.printf("pose %s, gyro %s, rate %f\n",
                    drive.getState().pose(),
                    gyro.getYawNWU(),
                    gyro.getYawRateNWU());
        drive.set(input);
        if (DEBUG)
            System.out.printf("pose %s, gyro %s, rate %f\n",
                    drive.getState().pose(),
                    gyro.getYawNWU(),
                    gyro.getYawRateNWU());
        stepTime();
        if (DEBUG)
            System.out.printf("pose %s, gyro %s, rate %f\n",
                    drive.getState().pose(),
                    gyro.getYawNWU(),
                    gyro.getYawRateNWU());
        drive.set(input);
        if (DEBUG)
            System.out.printf("pose %s, gyro %s, rate %f\n",
                    drive.getState().pose(),
                    gyro.getYawNWU(),
                    gyro.getYawRateNWU());
        stepTime();
        if (DEBUG)
            System.out.printf("pose %s, gyro %s, rate %f\n",
                    drive.getState().pose(),
                    gyro.getYawNWU(),
                    gyro.getYawRateNWU());

    }
}
