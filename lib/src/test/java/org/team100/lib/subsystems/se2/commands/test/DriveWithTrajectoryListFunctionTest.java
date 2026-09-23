package org.team100.lib.subsystems.se2.commands.test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;
import java.util.function.UnaryOperator;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.FusedEstimator;
import org.team100.lib.localization.StateEstimator;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.SimulatedGyro;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.SwerveLocal;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;
import org.team100.lib.trajectory.se2.examples.TrajectoryExamples;
import org.team100.lib.visualization.TrajectoryVisualization;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.system.DataLogManager;



class DriveWithTrajectoryListFunctionTest implements Timeless {

    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TrajectoryVisualization viz = new TrajectoryVisualization(logger);

    @BeforeEach
    void nolog() {
        DataLogManager.stop();
    }

    @Test
    void testSimple() throws IOException {

        LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        LoggerFactory fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // uses simulated modules
        SwerveModuleCollection collection = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20));
        Gyro gyro = new SimulatedGyro(logger, swerveKinodynamics, collection, 0);
        SwerveLocal swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        UnaryOperator<Twist2d> odometryNoise = UnaryOperator.identity();

        StateEstimator estimate = new FusedEstimator(
                logger, fieldLogger, swerveKinodynamics, odometryNoise, layout, gyro, swerveLocal);

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(
                logger,
                estimate,
                swerveLocal);

        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        // this initial step is required since the timebase is different?
        stepTime();
        Experiments.INSTANCE.override(Experiment.UseSwerveLimiter, true);
        ControllerSE2 control = ControllerFactorySE2.test(logger);
        DriveWithTrajectoryListFunction c = new DriveWithTrajectoryListFunction(
                logger,
                drive,
                control,
                x -> List.of(ex.line(x)),
                viz);
        c.initialize();
        assertEquals(0, drive.getState().pose().getX(), DELTA);
        c.execute();
        assertFalse(c.isDone());
        // the trajectory takes a little over 3s
        for (double t = 0; t < 4; t += TimedRobot100.LOOP_PERIOD_S) {
            stepTime();
            c.execute();
            drive.periodic(); // for updateOdometry
        }
        assertTrue(c.isDone());
        assertEquals(1.0, drive.getState().pose().getX(), 0.01);
    }
}
