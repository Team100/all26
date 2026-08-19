package org.team100.lib.subsystems.se2.commands.test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.subsystems.swerve.Fixture;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;
import org.team100.lib.trajectory.se2.examples.TrajectoryExamples;
import org.team100.lib.visualization.TrajectoryVisualization;
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
        Fixture fixture = new Fixture();
        List<TimingConstraint> constraints = new TimingConstraintFactory(fixture.swerveKinodynamics).allGood();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        // this initial step is required since the timebase is different?
        stepTime();
        Experiments.instance.testOverride(Experiment.UseSwerveLimiter, true);
        ControllerSE2 control = ControllerFactorySE2.test(logger);
        DriveWithTrajectoryListFunction c = new DriveWithTrajectoryListFunction(
                logger,
                fixture.drive,
                control,
                x -> List.of(ex.line(x)),
                viz);
        c.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), DELTA);
        c.execute();
        assertFalse(c.isDone());
        // the trajectory takes a little over 3s
        for (double t = 0; t < 4; t += TimedRobot100.LOOP_PERIOD_S) {
            stepTime();
            c.execute();
            fixture.drive.periodic(); // for updateOdometry
        }
        assertTrue(c.isDone());
        assertEquals(1.0, fixture.drive.getPose().getX(), 0.01);
    }
}
