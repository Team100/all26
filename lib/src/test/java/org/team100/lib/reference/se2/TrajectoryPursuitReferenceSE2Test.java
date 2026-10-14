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
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;
import org.team100.lib.trajectory.se2.examples.TrajectoryExamples;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class TrajectoryPursuitReferenceSE2Test {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
    List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
    PathSE2Factory pathFactory = new PathSE2Factory();
    TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
    TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);

    Translation2d position = new Translation2d();

    @Test
    void test0() {
        Cache.clear();
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        TrajectorySE2 t = ex.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        TrajectoryPursuitReferenceSE2 r = new TrajectoryPursuitReferenceSE2(
                logger, t, () -> position);
        {
            // at the starting point
            position = new Translation2d(0, 0);
            StateSE2 c = r.current();
            assertEquals(0, c.velocity().x(), DELTA);
            assertEquals(0, c.velocity().y(), DELTA);
            assertEquals(0, c.pose().getX(), DELTA);
            assertEquals(0, c.pose().getY(), DELTA);
            ControlSE2 n = r.next();
            assertEquals(0.033, n.velocity().x(), DELTA);
            assertEquals(0.0003, n.pose().getX(), DELTA);
        }

        {
            // test discretization, avoid "sticking" to the start
            position = new Translation2d(0.0003, 0);
            StateSE2 c = r.current();
            assertEquals(0.033, c.velocity().x(), DELTA);
            assertEquals(0, c.velocity().y(), DELTA);
            assertEquals(0, c.pose().getX(), DELTA);
            assertEquals(0, c.pose().getY(), DELTA);
            ControlSE2 n = r.next();
            assertEquals(0.065, n.velocity().x(), DELTA);
            assertEquals(0.0013, n.pose().getX(), DELTA);
        }

        {
            // off to the side, halfway down
            position = new Translation2d(0.5, 0.5);
            StateSE2 c = r.current();
            // feedforward from the nearest point
            assertEquals(1.274, c.velocity().x(), DELTA);
            // no feedforward toward the trajectory :(
            assertEquals(0, c.velocity().y(), DELTA);
            // closest point
            assertEquals(0.497, c.pose().getX(), DELTA);
            assertEquals(0, c.pose().getY(), DELTA);
            ControlSE2 n = r.next();
            assertEquals(1.249, n.velocity().x(), DELTA);
            assertEquals(0.522, n.pose().getX(), DELTA);
        }

    }

}
