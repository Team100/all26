package org.team100.lib.reference.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Cache;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;
import org.team100.lib.trajectory.se2.examples.TrajectoryExamples;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class TrajectoryReferenceTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
    List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
    PathSE2Factory pathFactory = new PathSE2Factory();
    TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
    TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);

    @Test
    void testSimple() {
        Cache.clear();
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        TrajectorySE2 t = ex.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
                TrajectoryReferenceSE2 r = new TrajectoryReferenceSE2(logger, t);
        // measurement is irrelevant
        r.initialize(new ModelSE2());
        {
            ModelSE2 c = r.current();
            assertEquals(0, c.velocity().x(), DELTA);
            assertEquals(0, c.pose().getX(), DELTA);
            ControlSE2 n = r.next();
            assertEquals(0.033, n.velocity().x(), DELTA);
            assertEquals(0.0003, n.pose().getX(), DELTA);
        }
        // no time step, nothing changes
        {
            ModelSE2 c = r.current();
            assertEquals(0, c.velocity().x(), DELTA);
            assertEquals(0, c.pose().getX(), DELTA);
            ControlSE2 n = r.next();
            assertEquals(0.033, n.velocity().x(), DELTA);
            // x is very small but not zero
            assertEquals(0.0003, n.pose().getX(), DELTA);
        }
        // stepping time gets the next references
        stepTime();
        {
            ModelSE2 c = r.current();
            assertEquals(0.033, c.velocity().x(), DELTA);
            assertEquals(0.0003, c.pose().getX(), DELTA);
            ControlSE2 n = r.next();
            assertEquals(0.065, n.velocity().x(), DELTA);
            assertEquals(0.0013, n.pose().getX(), DELTA);
        }
        // way in the future, we're at the end.
        for (int i = 0; i < 500; ++i) {
            stepTime();
        }
        {
            ModelSE2 c = r.current();
            assertEquals(0, c.velocity().x(), DELTA);
            assertEquals(1, c.pose().getX(), DELTA);
            ControlSE2 n = r.next();
            assertEquals(0, n.velocity().x(), DELTA);
            assertEquals(1, n.pose().getX(), DELTA);
        }

    }

}
