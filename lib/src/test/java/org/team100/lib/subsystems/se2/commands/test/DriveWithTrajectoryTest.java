package org.team100.lib.subsystems.se2.commands.test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;
import java.util.function.UnaryOperator;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.NudgingVisionUpdater;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.SimulatedGyro;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.se2.MockSubsystemSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.SwerveLocal;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;
import org.team100.lib.trajectory.se2.examples.TrajectoryExamples;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;
import org.team100.lib.visualization.TrajectoryVisualization;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class DriveWithTrajectoryTest implements Timeless {

    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TotalCurrentLog currentLog = new TotalCurrentLog(logger);
    private static final LoggerFactory fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TrajectoryVisualization viz = new TrajectoryVisualization(logger);

    @Test
    void testTrajectoryStart() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        TrajectorySE2 t = ex.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, t.sample(0).point().velocity(), DELTA);
        ControllerSE2 controller = ControllerFactorySE2.test(logger);

        // initially at rest
        MockSubsystemSE2 d = new MockSubsystemSE2(new ModelSE2());

        DriveWithTrajectory c = new DriveWithTrajectory(logger, d, controller, t, viz);

        stepTime();
        c.initialize();
        c.execute();
        // assertEquals(0.098, d.m_atRestSetpoint.x(), DELTA);
        // assertEquals(0, d.m_atRestSetpoint.y(), DELTA);
        // assertEquals(0, d.m_atRestSetpoint.theta(), DELTA);

        // we don't advance because we're still steering.
        // this next-setpoint is from "preview"
        // and our current setpoint is equal to the measurement.
        stepTime();
        c.execute();
        // assertEquals(0.098, d.m_atRestSetpoint.x(), DELTA);
        // assertEquals(0, d.m_atRestSetpoint.y(), DELTA);
        // assertEquals(0, d.m_atRestSetpoint.theta(), DELTA);

        stepTime();
        c.execute();
        assertEquals(0.102, d.m_setpoint.x().v(), DELTA);
        assertEquals(0, d.m_setpoint.y().v(), DELTA);
        assertEquals(0, d.m_setpoint.theta().v(), DELTA);

        // more normal driving
        stepTime();
        c.execute();
        assertEquals(0.139, d.m_setpoint.x().v(), DELTA);
        assertEquals(0, d.m_setpoint.y().v(), DELTA);
        assertEquals(0, d.m_setpoint.theta().v(), DELTA);

        // etc
        stepTime();
        c.execute();
        assertEquals(0.179, d.m_setpoint.x().v(), DELTA);
        assertEquals(0, d.m_setpoint.y().v(), DELTA);
        assertEquals(0, d.m_setpoint.theta().v(), DELTA);
    }

    @Test
    void testTrajectoryDone() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        TrajectorySE2 t = ex.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, t.sample(0).point().velocity(), DELTA);
        ControllerSE2 controller = ControllerFactorySE2.test(logger);

        // initially at rest
        MockSubsystemSE2 d = new MockSubsystemSE2(new ModelSE2());

        DriveWithTrajectory c = new DriveWithTrajectory(logger, d, controller, t, viz);
        c.initialize();

        for (int i = 0; i < 100; ++i) {
            stepTime();
            c.execute();
            // we have magically reached the end
            d.m_state = new ModelSE2(new Pose2d(1, 0, Rotation2d.kZero));
        }
        assertTrue(c.isDone());

    }

    /**
     * Use a real drivetrain to observe the effect on the motors etc.
     * 
     * @throws IOException
     */
    @Test
    void testRealDrive() throws IOException {

        // this test depends on the behavior of the setpoint generator, so make sure
        // it's on (otherwise it's in whatever state the previous test left it)
        Experiments.instance.testOverride(Experiment.UseSwerveLimiter, true);
        // 1m along +x, no rotation.
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        SwerveModuleCollection collection = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20), swerveKinodynamics);
        collection.reset();
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        TrajectorySE2 trajectory = ex.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, trajectory.sample(0).point().velocity(), DELTA);
        ControllerSE2 controller = ControllerFactorySE2.test(logger);

        Gyro gyro = new SimulatedGyro(logger, swerveKinodynamics, collection, 0);
        SwerveHistory history = new SwerveHistory(
                logger,
                swerveKinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                SwerveModulePositions.kZero(),
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0); // initial time is zero here for testing
        OdometryUpdater odometryUpdater = new OdometryUpdater(
                logger, swerveKinodynamics, gyro, history, collection::positions, UnaryOperator.identity());
        odometryUpdater.reset(Pose2d.kZero, IsotropicNoiseSE2.high(), 0);

        NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(
                logger, history, odometryUpdater);

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, fieldLogger, layout, history, visionUpdater,MatchState::getAlliance);
        FreshSwerveEstimate estimate = new FreshSwerveEstimate(
            localizer::update, odometryUpdater::update, history);
        SwerveLocal swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(
                logger,
                odometryUpdater,
                estimate,
                swerveLocal);

        // initially at rest
        assertEquals(0, collection.states().frontLeft().speed(), DELTA);
        assertEquals(0, collection.states().frontLeft().angle().get().getRadians(), DELTA);

        DriveWithTrajectory command = new DriveWithTrajectory(
                logger, drive, controller, trajectory, viz);
        stepTime();
        command.initialize();

        command.execute();
        // but that output is not available until after takt.
        assertEquals(0, collection.states().frontLeft().speed(), DELTA);
        assertEquals(0, collection.states().frontLeft().angle().get().getRadians(), DELTA);

        // drive normally more
        stepTime();
        command.execute();
        // this is the output from the previous takt
        assertEquals(0.033, collection.states().frontLeft().speed(), DELTA);
        assertEquals(0, collection.states().frontLeft().angle().get().getRadians(), DELTA);

        // etc
        stepTime();
        command.execute();
        assertEquals(0.064, collection.states().frontLeft().speed(), DELTA);
        assertEquals(0, collection.states().frontLeft().angle().get().getRadians(), DELTA);
    }

}
