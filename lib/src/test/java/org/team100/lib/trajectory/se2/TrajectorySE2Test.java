package org.team100.lib.trajectory.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.spline.se2.SplineSE2;
import org.team100.lib.spline.se2.SplineSE2Factory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;
import org.team100.lib.util.ChartUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

class TrajectorySE2Test implements Timeless {
    private static final double DELTA = 0.001;
    private static final boolean DEBUG = false;

    @Test
    void testPreviewAndAdvance() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest();
        Pose2d start = Pose2d.kZero;
        Pose2d end = start.plus(new Transform2d(1, 0, Rotation2d.kZero));

        Translation2d currentTranslation = start.getTranslation();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(start, DirectionSE2.irrotational(angleToGoal), 1),
                new WaypointSE2(end, DirectionSE2.irrotational(angleToGoal), 1));

        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).fast();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);

        TrajectorySE2 trajectory = planner.restToRest(waypoints);

        TrajectorySE2Entry sample = trajectory.sample(0);
        assertEquals(0, sample.point().point().waypoint().pose().getTranslation().getX(), DELTA);

        sample = trajectory.sample(1);
        assertEquals(1, sample.point().point().waypoint().pose().getTranslation().getX(), DELTA);

        sample = trajectory.sample(2);
        assertEquals(1, sample.point().point().waypoint().pose().getTranslation().getX(), DELTA);

        sample = trajectory.sample(3);
        assertEquals(1, sample.point().point().waypoint().pose().getTranslation().getX(), DELTA);
    }

    @Test
    void testSample() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest3();
        Pose2d start = Pose2d.kZero;
        Pose2d end = start.plus(new Transform2d(1, 0, Rotation2d.kZero));

        Translation2d currentTranslation = start.getTranslation();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(start, DirectionSE2.irrotational(angleToGoal), 1),
                new WaypointSE2(end, DirectionSE2.irrotational(angleToGoal), 1));

        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).fast();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);

        TrajectorySE2 trajectory = planner.restToRest(waypoints);
        if (DEBUG)
            trajectory.dump();

        assertEquals(1.415, trajectory.duration(), DELTA);
        TrajectorySE2Entry sample = trajectory.sample(0);
        assertEquals(0, sample.point().point().waypoint().pose().getTranslation().getX(), DELTA);
        sample = trajectory.sample(1);
        assertEquals(0.827, sample.point().point().waypoint().pose().getTranslation().getX(), DELTA);
        sample = trajectory.sample(2);
        assertEquals(1, sample.point().point().waypoint().pose().getTranslation().getX(), DELTA);
    }

    /**
     * This is to hold the sample invariant while adding the index.
     */
    @Test
    void testSampleThoroughly() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                Rotation2d.kZero),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(1, 0, 0), 1));

        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest3();
        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).fast();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);

        TrajectorySE2 trajectory = planner.restToRest(waypoints);
        if (DEBUG)
            System.out.println(trajectory);

        if (DEBUG) {
            for (double t = 0; t < 1.5; t += 0.1) {
                System.out.printf("%5.3f %s\n", t, trajectory.sample(t));
            }
        }

        assertEquals(1.415, trajectory.duration(), DELTA);
        check(trajectory, 0.0, 0.000);
        check(trajectory, 0.1, 0.010);
        check(trajectory, 0.2, 0.040);
        check(trajectory, 0.3, 0.090);
        check(trajectory, 0.4, 0.161);
        check(trajectory, 0.5, 0.250);
        check(trajectory, 0.6, 0.360);
        check(trajectory, 0.7, 0.490);
        check(trajectory, 0.8, 0.622);
        check(trajectory, 0.9, 0.734);
        check(trajectory, 1.0, 0.827);
        check(trajectory, 1.1, 0.901);
        check(trajectory, 1.2, 0.954);
        check(trajectory, 1.3, 0.987);
        check(trajectory, 1.4, 1.000);
        check(trajectory, 1.5, 1.000);
    }

    private void check(TrajectorySE2 trajectory, double t, double x) {
        assertEquals(x, trajectory.sample(t).point().point().waypoint().pose().getTranslation().getX(), DELTA);
    }

    /**
     * Does the index help? No.
     * 
     * Does interpolation help, relative to just sampling the spline directly? No.
     * 
     * There's no need to run this all the time
     */
    // disable to speed up tests
    // @Test
    void testSamplePerformance() {
        WaypointSE2 p0 = new WaypointSE2(new Pose2d(new Translation2d(), Rotation2d.kZero),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 p1 = new WaypointSE2(new Pose2d(new Translation2d(10, 0), Rotation2d.kCCW_Pi_2),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 p2 = new WaypointSE2(new Pose2d(new Translation2d(10, 10), Rotation2d.kPi),
                new DirectionSE2(1, 0, 0), 1);
        List<WaypointSE2> waypoints = List.of(p0, p1, p2);
        List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);

        int reps = 100000;
        int times = 10;

        // SAMPLE SPLINE DIRECTLY (200 ns)

        SplineSE2 spline = new SplineSE2(p0, p1);

        long start = System.nanoTime();
        for (int rep = 0; rep < reps; ++rep) {
            for (int t = 0; t < times; ++t) {
                spline.entry(0.1 * t).point();
            }
        }
        long end = System.nanoTime();
        long duration = end - start;
        if (DEBUG)
            System.out.printf("SPLINE duration (ns) total (ms) %.0f per sample (ns) %.2f\n",
                    0.000001 * duration, (double) duration / (reps * times));

        // INTERPOLATE SPLINE POINTS (170 ns)

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.02, 0.1);
        PathSE2 path = pathFactory.get(splines);
        assertEquals(22.734, path.distance(path.length() - 1), 0.001);

        start = System.nanoTime();
        for (int rep = 0; rep < reps; ++rep) {
            for (int t = 0; t < times; ++t) {
                path.sample(0.1 * t);
            }
        }
        end = System.nanoTime();
        duration = end - start;
        if (DEBUG)
            System.out.printf("PATH duration (ns) total (ms) %.0f per sample (ns) %.2f\n",
                    0.000001 * duration, (double) duration / (reps * times));

        /////////////
        // INTERPOLATE TRAJECTORY POINTS (335 ns)

        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest3();
        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).fast();
        TrajectorySE2Factory generator = new TrajectorySE2Factory(constraints);

        TrajectorySE2 trajectory = generator.fromPath(path, 0, 0);
        List<VectorSeries> series = new TrajectorySE2ToVectorSeries(1).convert(trajectory);
        ChartUtil.plotOverlay(series, 100);

        assertEquals(313, trajectory.length());

        start = System.nanoTime();
        for (int rep = 0; rep < reps; ++rep) {
            for (int t = 0; t < times; ++t) {
                trajectory.sample(0.1 * t);
            }
        }
        end = System.nanoTime();
        duration = end - start;
        if (DEBUG)
            System.out.printf("TRAJECTORY duration (ns) total (ms) %.0f per sample (ns) %.2f\n",
                    0.000001 * duration, (double) duration / (reps * times));

    }

}
