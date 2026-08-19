package org.team100.frc2025;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Stream;

import org.jfree.data.xy.VectorSeries;
import org.junit.jupiter.api.Test;
import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.Swerve.Auto.GoToCoralStation;
import org.team100.frc2025.field.FieldConstants2025;
import org.team100.frc2025.field.FieldConstants2025.CoralStation;
import org.team100.frc2025.field.FieldConstants2025.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.kinematics.prr.PRRKinematics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.TrajectorySE2ToVectorSeries;
import org.team100.lib.trajectory.se2.constraint.ConstantConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TorqueConstraint;
import org.team100.lib.trajectory.se2.constraint.YawRateConstraint;
import org.team100.lib.util.ChartUtil;

import org.wpilib.math.geometry.Pose2d;

/** Show some trajectories from 2025 in a vector series chart. */
public class TrajectoryGallery {
    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();

    @Test
    void testGoToCoralStation1() {
        List<VectorSeries> right = series(CoralStation.RIGHT, ReefPoint.F, ScoringLevel.L4);
        ChartUtil.plotOverlay(right, 100);
    }

    @Test
    void testGoToCoralStation2() {
        List<VectorSeries> left = series(CoralStation.LEFT, ReefPoint.I, ScoringLevel.L4);
        ChartUtil.plotOverlay(left, 100);
    }

    @Test
    void testGoToCoralStation3() {
        List<VectorSeries> iLeft = series(CoralStation.LEFT, ReefPoint.I, ScoringLevel.L4);
        List<VectorSeries> kLeft = series(CoralStation.LEFT, ReefPoint.K, ScoringLevel.L4);
        List<VectorSeries> lLeft = series(CoralStation.LEFT, ReefPoint.L, ScoringLevel.L4);
        List<VectorSeries> fRight = series(CoralStation.RIGHT, ReefPoint.F, ScoringLevel.L4);
        List<VectorSeries> dRight = series(CoralStation.RIGHT, ReefPoint.D, ScoringLevel.L4);
        List<VectorSeries> cRight = series(CoralStation.RIGHT, ReefPoint.C, ScoringLevel.L4);
        ChartUtil.plotOverlay(Stream.of(iLeft, kLeft, lLeft, fRight, dRight, cRight)
                .flatMap(Collection::stream).toList(), 100);
    }

    @Test
    void testCalgames() {
        List<VectorSeries> homeToL2 = series(
                DirectionSE2.irrotational(1.5),
                WaypointSE2.irrotational(CalgamesMech.L2, 1.5, 1.2));
        List<VectorSeries> homeToL3 = series(
                DirectionSE2.irrotational(0.8),
                WaypointSE2.irrotational(CalgamesMech.L3, 1.5, 1.2));
        List<VectorSeries> homeToL4 = series(
                DirectionSE2.irrotational(0.1),
                WaypointSE2.irrotational(CalgamesMech.L4, 1.5, 1.2));
        List<VectorSeries> homeToAlgaeL2 = series(
                DirectionSE2.irrotational(1.5),
                WaypointSE2.irrotational(CalgamesMech.ALGAE_L2, 1.5, 1.2));
        List<VectorSeries> homeToAlgaeL3 = series(
                DirectionSE2.irrotational(0),
                WaypointSE2.irrotational(CalgamesMech.ALGAE_L3, 1.5, 1.2));
        List<VectorSeries> homeToBarge = series(
                DirectionSE2.irrotational(0),
                WaypointSE2.irrotational(CalgamesMech.BARGE, -1, 1.2));

        ChartUtil.plotOverlay(Stream.of(
                homeToL2, homeToL3, homeToL4, homeToAlgaeL2, homeToAlgaeL3, homeToBarge)
                .flatMap(Collection::stream).toList(), 500);

    }

    private List<VectorSeries> series(DirectionSE2 m_course, WaypointSE2 m_goal) {
        // homeToL2
        List<TimingConstraint> c = new ArrayList<>();
        // These are known to work, but suboptimal.
        c.add(new ConstantConstraint(10, 5));
        c.add(new YawRateConstraint(10, 5));
        // This is new
        c.add(new TorqueConstraint(20));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(c);
        PathSE2Factory pathFactory = new PathSE2Factory(0.05, 0.01, 0.1);
        TrajectorySE2Planner m_planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);

        PRRKinematics m_kinematics = new PRRKinematics(0.5, 0.343, PRRKinematics.Solver.ANALYTIC);
        Pose2d m_home = m_kinematics.forward(CalgamesMech.HOME);
        WaypointSE2 start = new WaypointSE2(m_home, m_course, 1);
        TrajectorySE2 m_trajectory = m_planner.restToRest(List.of(start, m_goal));
        List<VectorSeries> series = new TrajectorySE2ToVectorSeries(0.1).convert(m_trajectory);
        return series;
    }

    private List<VectorSeries> series(CoralStation coralStation, ReefPoint reefPoint, ScoringLevel scoringLevel) {
        GoToCoralStation toStation = new GoToCoralStation(swerveKinodynamics, coralStation, 0.5);
        // the start is the goal from the previous maneuver
        Pose2d start = FieldConstants2025.makeGoal(scoringLevel, reefPoint);
        TrajectorySE2 trajectory = toStation.apply(start);
        return new TrajectorySE2ToVectorSeries(0.1).convert(trajectory);
    }

}
