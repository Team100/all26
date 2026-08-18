package org.team100.lib.spline.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2ToVectorSeries;
import org.team100.lib.trajectory.se2.constraint.CapsizeAccelerationConstraint;
import org.team100.lib.trajectory.se2.constraint.ConstantConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.util.ChartUtil;
import org.team100.lib.util.Math100;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

class SplineSE2Test implements Timeless {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    @Test
    void testCurvature() {
        // straight line, zero curvature.
        SplineSE2 s = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1));
        assertEquals(0, s.entry(0.5).point().k(), DELTA);

        // left turn
        s = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 1),
                                new Rotation2d()),
                        new DirectionSE2(0, 1, 0), 1));
        assertEquals(0.950, s.entry(0.5).point().k(), DELTA);

    }

    @Test
    void testCourse() {
        Rotation2d course = new Rotation2d(Math.PI / 4);
        Translation2d t = new Translation2d(1, 0).rotateBy(course);
        assertEquals(0.707, t.getX(), DELTA);
        assertEquals(0.707, t.getY(), DELTA);
    }

    @Test
    void testLinear() {
        SplineSE2 s = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1));

        SplineSE2ToVectorSeries splineConverter = new SplineSE2ToVectorSeries(0.1);
        List<VectorSeries> series = splineConverter.convert(List.of(s));
        ChartUtil.plotOverlay(series, 500);

        Translation2d t = s.entry(0).point().waypoint().pose().getTranslation();
        assertEquals(0, t.getX(), DELTA);
        t = s.entry(1).point().waypoint().pose().getTranslation();
        assertEquals(1, t.getX(), DELTA);
        PathSE2Point p = s.entry(0).point();
        assertEquals(0, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.waypoint().course().headingRate(), DELTA);
        p = s.entry(1).point();
        assertEquals(1, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.waypoint().course().headingRate(), DELTA);
    }

    @Test
    void testLinear2() {
        SplineSE2 s = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(2, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1));

        SplineSE2ToVectorSeries splineConverter = new SplineSE2ToVectorSeries(0.1);
        List<VectorSeries> series = splineConverter.convert(List.of(s));
        ChartUtil.plotOverlay(series, 500);

        Translation2d t = s.entry(0).point().waypoint().pose().getTranslation();
        assertEquals(0, t.getX(), DELTA);
        t = s.entry(1).point().waypoint().pose().getTranslation();
        assertEquals(2, t.getX(), DELTA);
        PathSE2Point p = s.entry(0).point();
        assertEquals(0, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.waypoint().course().headingRate(), DELTA);
        p = s.entry(1).point();
        assertEquals(2, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.waypoint().course().headingRate(), DELTA);
    }

    @Test
    void testRotationSoft() {
        // move ahead 1m while rotation 1 rad to the left
        // this has no rotation at the ends.
        // the rotation rate is zero at the ends and much higher in the middle.
        SplineSE2 s = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(1)),
                        new DirectionSE2(1, 0, 0), 1.2));

        SplineSE2ToVectorSeries splineConverter = new SplineSE2ToVectorSeries(0.1);
        List<VectorSeries> series = splineConverter.convert(List.of(s));
        ChartUtil.plotOverlay(series, 500);

        // now that the magic numbers apply to the rotational scaling, the heading rate
        // is constant.
        Translation2d t = s.entry(0).point().waypoint().pose().getTranslation();
        assertEquals(0, t.getX(), DELTA);
        t = s.entry(1).point().waypoint().pose().getTranslation();
        assertEquals(1, t.getX(), DELTA);

        PathSE2Point p = s.entry(0).point();
        assertEquals(0, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getRadians(), DELTA);
        // initial rotation rate is zero
        assertEquals(0, p.waypoint().course().headingRate(), DELTA);

        p = s.entry(0.5).point();
        assertEquals(0.5, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0.5, p.waypoint().pose().getRotation().getRadians(), DELTA);
        // high rotation rate in the middle
        assertEquals(2.273, p.waypoint().course().headingRate(), DELTA);

        p = s.entry(1).point();
        assertEquals(1, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(1, p.waypoint().pose().getRotation().getRadians(), DELTA);
        // rotation rate is zero at the end
        assertEquals(0, p.waypoint().course().headingRate(), DELTA);

    }

    @Test
    void testRotationFast() {
        // move ahead 1m while rotation 1 rad to the left
        // this has lots of rotation at the ends
        // the "spatial" rotation rate is constant, i.e.
        // rotation and translation speed are proportional.
        SplineSE2 s = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 1), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(1)),
                        new DirectionSE2(1, 0, 1), 1));

        SplineSE2ToVectorSeries splineConverter = new SplineSE2ToVectorSeries(0.1);
        List<VectorSeries> series = splineConverter.convert(List.of(s));
        ChartUtil.plotOverlay(series, 500);

        // now that the magic numbers apply to the rotational scaling, the heading rate
        // is constant.
        Translation2d t = s.entry(0).point().waypoint().pose().getTranslation();
        assertEquals(0, t.getX(), DELTA);
        t = s.entry(1).point().waypoint().pose().getTranslation();
        assertEquals(1, t.getX(), DELTA);

        PathSE2Point p = s.entry(0).point();
        assertEquals(0, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getRadians(), DELTA);
        assertEquals(1, p.waypoint().course().headingRate(), DELTA);

        p = s.entry(0.5).point();
        assertEquals(0.5, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0.5, p.waypoint().pose().getRotation().getRadians(), DELTA);
        assertEquals(1, p.waypoint().course().headingRate(), DELTA);

        p = s.entry(1).point();
        assertEquals(1, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(1, p.waypoint().pose().getRotation().getRadians(), DELTA);
        assertEquals(1, p.waypoint().course().headingRate(), DELTA);
    }

    @Test
    void testRotation2() {
        // Make sure the rotation goes over +/- pi
        SplineSE2 s = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d(2.5)),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(-2.5)),
                        new DirectionSE2(1, 0, 0), 1));

        SplineSE2ToVectorSeries splineConverter = new SplineSE2ToVectorSeries(0.1);
        List<VectorSeries> series = splineConverter.convert(List.of(s));
        ChartUtil.plotOverlay(series, 500);
    }

    /** Turning in place splines do not work. */
    @Test
    void spin() {
        double scale = 0.9;
        assertThrows(IllegalArgumentException.class, () -> new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(0, 0, 1), scale),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                Rotation2d.kCCW_90deg),
                        new DirectionSE2(0, 0, 1), scale)));
    }

    /**
     * Four splines that make an approximate circle.
     * 
     * The heading rate is pretty constant, looking at the origin pretty closely.
     * 
     * The path curvature is not constant, because that's how our splines work: the
     * curvature is always zero at the ends. It does mostly range from about 0.75 to
     * about 1.25, which is kinda close?
     */
    @Test
    void testCircle() {
        double scale = 1.3;
        WaypointSE2 p0 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), Rotation2d.k180deg),
                new DirectionSE2(0, 1, 1), scale);
        WaypointSE2 p1 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 1), Rotation2d.kCW_90deg),
                new DirectionSE2(-1, 0, 1), scale);
        WaypointSE2 p2 = new WaypointSE2(
                new Pose2d(new Translation2d(-1, 0), Rotation2d.kZero),
                new DirectionSE2(0, -1, 1), scale);
        WaypointSE2 p3 = new WaypointSE2(
                new Pose2d(new Translation2d(0, -1), Rotation2d.kCCW_90deg),
                new DirectionSE2(1, 0, 1), scale);
        SplineSE2 s0 = new SplineSE2(p0, p1);
        SplineSE2 s1 = new SplineSE2(p1, p2);
        SplineSE2 s2 = new SplineSE2(p2, p3);
        SplineSE2 s3 = new SplineSE2(p3, p0);
        List<SplineSE2> splines = List.of(s0, s1, s2, s3);
        checkCircle(splines, 0.008, 0.006);
        List<VectorSeries> series = new SplineSE2ToVectorSeries(0.1).convert(splines);
        ChartUtil.plotOverlay(series, 300);
    }

    @Test
    void testDheading() {
        // does the spline-derived dheading match the post-hoc one?
        double scale = 0.9;
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), Rotation2d.k180deg),
                new DirectionSE2(0, 1, 1), scale);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 1), Rotation2d.kCW_90deg),
                new DirectionSE2(-1, 0, 1), scale);
        SplineSE2 spline = new SplineSE2(w0, w1);
        PathSE2Point p0 = spline.entry(0.0).point();
        if (DEBUG)
            System.out.println(
                    "s, p0_heading_rate, p0_curvature, distance, post_hoc_heading_rate, post_hoc_curvature, post_hoc_heading_rate2, post_hoc_curvature2");
        for (double s = 0.01; s <= 1.0; s += 0.01) {
            PathSE2Point p1 = spline.entry(s).point();
            double cartesianDistance = p1.distanceCartesian(p0);
            Rotation2d heading0 = p0.waypoint().pose().getRotation();
            Rotation2d heading1 = p1.waypoint().pose().getRotation();
            double dheading = heading1.minus(heading0).getRadians();
            DirectionSE2 course0 = p0.waypoint().course();
            DirectionSE2 course1 = p1.waypoint().course();
            double curve = Metrics.translationalNorm(course1.minus(course0));
            // this value matches the intrinsic one since it just uses
            // cartesian distance in the denominator.
            double dheadingDx2 = dheading / cartesianDistance;
            double curveDx2 = curve / cartesianDistance;

            if (DEBUG)
                System.out.printf(
                        "%5.3f, %5.3f, %5.3f, %5.3f, %5.3f \n",
                        s, p0.waypoint().course().headingRate(), p0.k(),
                        dheadingDx2, curveDx2);
            p0 = p1;
        }

    }

    private void checkCircle(List<SplineSE2> splines, double rangeError, double azimuthError) {
        double actualRangeError = 0;
        double actualAzimuthError = 0;
        if (DEBUG)
            System.out.println("s, x, y, k, dh");
        for (SplineSE2 spline : splines) {
            for (double s = 0; s < 0.99; s += 0.01) {
                Pose2d p = spline.entry(s).point().waypoint().pose();
                // the position should be on the circle
                double range = p.getTranslation().getNorm();
                actualRangeError = Math.max(actualRangeError, Math.abs(1.0 - range));

                // the heading should point to the origin all the time.
                Rotation2d angleFromOrigin = p.getTranslation().unaryMinus().getAngle();
                Rotation2d error = angleFromOrigin.minus(p.getRotation());
                // there's about 2 degrees of error here because the spline is not quite a
                // circle.
                // 3/10/25 i made generation coarser so it's less accurate.
                actualAzimuthError = Math.max(actualAzimuthError, Math.abs(error.getRadians()));
                double k = spline.entry(s).point().k();
                double dh = spline.entry(s).point().waypoint().course().headingRate();
                if (DEBUG)
                    System.out.printf("%f, %f, %f, %f, %f\n", s, p.getX(), p.getY(), k, dh);
            }
        }
        assertEquals(0, actualRangeError, rangeError,
                String.format("range actual %f allowed %f", actualRangeError, rangeError));
        assertEquals(0, actualAzimuthError, azimuthError,
                String.format("azimuth actual %f expected %f", actualAzimuthError, azimuthError));

    }

    @Test
    void testLine() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), Rotation2d.kZero),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), new Rotation2d(1)),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 w2 = new WaypointSE2(
                new Pose2d(new Translation2d(2, 0), Rotation2d.k180deg),
                new DirectionSE2(1, 0, 0), 1);
        // turn a bit to the left
        SplineSE2 s0 = new SplineSE2(w0, w1);
        // turn much more to the left
        SplineSE2 s1 = new SplineSE2(w1, w2);

        List<SplineSE2> splines = List.of(s0, s1);
        List<VectorSeries> series = new SplineSE2ToVectorSeries(0.1).convert(splines);
        ChartUtil.plotOverlay(series, 500);
    }

    /**
     * A kinda-realistic test path:
     * 
     * * start facing towards the driver
     * * back up
     * * rotate towards +y, also drive towards +y
     */
    @Test
    void testPath0() {
        double scale = 0.7;
        WaypointSE2 p0 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d(-1, 0)),
                new DirectionSE2(1, 0, 0), scale);
        WaypointSE2 p1 = new WaypointSE2(
                new Pose2d(new Translation2d(0.707, 0.293), new Rotation2d(-1, 1)),
                new DirectionSE2(1, 1, -1), scale);
        WaypointSE2 p2 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 1), new Rotation2d(0, 1)),
                new DirectionSE2(0, 1, 0), scale);
        if (DEBUG)
            System.out.println("s01");
        SplineSE2 s01 = new SplineSE2(p0, p1);
        if (DEBUG)
            System.out.println("s12");
        SplineSE2 s12 = new SplineSE2(p1, p2);
        List<SplineSE2> splines = List.of(s01, s12);
        List<VectorSeries> series = new SplineSE2ToVectorSeries(0.1).convert(splines);
        ChartUtil.plotOverlay(series, 500);
    }

    @Test
    void testMismatchedXYDerivatives() {
        // because path generation never looks across spline boundaries,
        // it is possible to make sharp corners at the "knots."

        // this goes straight ahead to (1,0)
        // derivatives point straight ahead
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), Rotation2d.kZero),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), Rotation2d.kZero),
                new DirectionSE2(1, 0, 0), 1);
        SplineSE2 s0 = new SplineSE2(w0, w1);
        // this is a sharp turn to the left
        // derivatives point to the left
        WaypointSE2 w2 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), Rotation2d.kZero),
                new DirectionSE2(0, 1, 0), 1);
        WaypointSE2 w3 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 1), Rotation2d.kZero),
                new DirectionSE2(0, 1, 0), 1);
        SplineSE2 s1 = new SplineSE2(w2, w3);
        List<SplineSE2> splines = List.of(s0, s1);

        List<VectorSeries> series = new SplineSE2ToVectorSeries(0.1).convert(splines);
        ChartUtil.plotOverlay(series, 500);

        for (SplineSE2 s : splines) {
            if (DEBUG)
                System.out.printf("spline %s\n", s);
        }

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.05, 0.05);
        PathSE2 path = pathFactory.get(splines);
        if (DEBUG)
            System.out.printf("path %s\n", path);
        List<TimingConstraint> constraints = List.of(new ConstantConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        TrajectorySE2 traj = trajectoryFactory.fromPath(path, 0, 0);
        if (DEBUG)
            traj.dump();
        List<VectorSeries> series2 = new TrajectorySE2ToVectorSeries(0.1).convert(traj);
        ChartUtil.plotOverlay(series2, 500);
    }

    @Test
    void testEntryVelocity() {

        // radius is 1 m.
        SplineSE2 s0 = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(new Translation2d(0, -1), Rotation2d.kZero),
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(new Translation2d(1, 0), Rotation2d.kZero),
                        new DirectionSE2(0, 1, 0), 1.2));
        if (DEBUG) {
            for (double s = 0; s < 1; s += 0.03) {
                Pose2d pose = s0.entry(s).point().waypoint().pose();
                System.out.printf("%5.3f %5.3f\n", pose.getX(), pose.getY());
            }
        }

        List<SplineSE2> splines = List.of(s0);

        SplineSE2ToVectorSeries splineConverter = new SplineSE2ToVectorSeries(0.1);
        List<VectorSeries> series = splineConverter.convert(splines);
        ChartUtil.plotOverlay(series, 500);

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.05, 0.05);
        PathSE2 path = pathFactory.get(splines);
        if (DEBUG) {
            for (int i = 0; i < path.length(); ++i) {
                PathSE2Point p = path.getEntry(i).point();
                System.out.printf("%5.3f %5.3f\n", p.waypoint().pose().getTranslation().getX(),
                        p.waypoint().pose().getTranslation().getY());
            }
        }
        if (DEBUG) {
            for (int i = 0; i < path.length(); ++i) {
                System.out.printf("%5.3f %5.3f\n",
                        path.getEntry(i).point().waypoint().pose().getTranslation().getX(),
                        path.getEntry(i).point().waypoint().pose().getTranslation().getY());
            }
        }

        // if we enter a circle at the capsize velocity, we should continue at that same
        // speed.
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest();
        // centripetal accel is 8.166 m/s^2
        assertEquals(8.166666, limits.getMaxCapsizeAccelM_S2(), 1e-6);
        List<TimingConstraint> constraints = List.of(
                new CapsizeAccelerationConstraint(limits, 1.0));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        // speed
        // a = v^2/r so v = sqrt(ar) = 2.858
        TrajectorySE2 trajectory = trajectoryFactory.fromPath(path, 2.858, 2.858);

        TrajectorySE2ToVectorSeries converter = new TrajectorySE2ToVectorSeries(0.1);

        List<VectorSeries> series2 = converter.convert(trajectory);
        ChartUtil.plotOverlay(series2, 500);

        if (DEBUG)
            System.out.printf("trajectory %s\n", trajectory);
    }

    @Test
    void testHeadingRate() {
        // note spline rotation rate is not constant, to make it more interesting
        SplineSE2 spline = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(1)),
                        new DirectionSE2(1, 0, 1), 1));

        double splineHR = spline.entry(0.5).point().waypoint().course().headingRate();
        assertEquals(1.388, splineHR, DELTA);
        Pose2d p0 = spline.entry(0.49).point().waypoint().pose();
        Pose2d p1 = spline.entry(0.51).point().waypoint().pose();
        double discreteHR = GeometryUtil.headingRatio(p0, p1);
        assertEquals(0.811, discreteHR, DELTA);
    }

    @Test
    void testCurvature1() {
        SplineSE2 spline = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 1),
                                new Rotation2d()),
                        new DirectionSE2(0, 1, 0), 1));
        // verify one point
        {
            double splineCurvature = spline.entry(0.5).point().k();
            assertEquals(0.950, splineCurvature, DELTA);
            Pose2d p0 = spline.entry(0.49).point().waypoint().pose();
            Pose2d p1 = spline.entry(0.50).point().waypoint().pose();
            Pose2d p2 = spline.entry(0.51).point().waypoint().pose();
            double mengerCurvature = GeometryUtil.mengerCurvature(
                    p0.getTranslation(), p1.getTranslation(), p2.getTranslation());
            assertEquals(0.950, mengerCurvature, DELTA);
        }
        // verify all the points
        double DS = 0.01;
        for (double s = DS; s <= 1 - DS; s += DS) {
            double splineCurvature = spline.entry(s).point().k();
            Pose2d p0 = spline.entry(s - DS).point().waypoint().pose();
            Pose2d p1 = spline.entry(s).point().waypoint().pose();
            Pose2d p2 = spline.entry(s + DS).point().waypoint().pose();
            double mengerCurvature = GeometryUtil.mengerCurvature(
                    p0.getTranslation(), p1.getTranslation(), p2.getTranslation());
            if (DEBUG)
                System.out.printf("%f %f %f %f\n", s, splineCurvature, mengerCurvature,
                        splineCurvature - mengerCurvature);
            // error scales with ds.
            assertEquals(splineCurvature, mengerCurvature, 0.001);
        }
    }

    @Test
    void testCurvature2() {
        // no curve
        SplineSE2 spline = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1));
        double splineCurvature = spline.entry(0.5).point().k();
        assertEquals(0, splineCurvature, DELTA);
        Pose2d p0 = spline.entry(0.49).point().waypoint().pose();
        Pose2d p1 = spline.entry(0.50).point().waypoint().pose();
        Pose2d p2 = spline.entry(0.51).point().waypoint().pose();
        double mengerCurvature = GeometryUtil.mengerCurvature(
                p0.getTranslation(), p1.getTranslation(), p2.getTranslation());
        assertEquals(0, mengerCurvature, DELTA);
    }

    @Test
    void testCurvature3() {
        // turn in place
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d()),
                new DirectionSE2(0, 0, 1), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d(1)),
                new DirectionSE2(0, 0, 1), 1);
        assertThrows(IllegalArgumentException.class, () -> new SplineSE2(w0, w1));
    }

    @Test
    void testMulti0() {
        WaypointSE2 a = new WaypointSE2(
                new Pose2d(new Translation2d(0, 2), new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 b = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 c = new WaypointSE2(
                new Pose2d(new Translation2d(2, 2), new Rotation2d()),
                new DirectionSE2(0, 1, 0), 1);

        List<SplineSE2> splines = new ArrayList<>();
        splines.add(new SplineSE2(a, b));
        splines.add(new SplineSE2(b, c));

        List<VectorSeries> series = new SplineSE2ToVectorSeries(0.1).convert(splines);
        List<VectorSeries> series2 = new SplineSE2ToVectorSeries(0.25).curvature(splines);
        List<VectorSeries> all = new ArrayList<>();
        all.addAll(series);
        all.addAll(series2);
        ChartUtil.plotOverlay(all, 300);

    }

    @Test
    void testMulti1() {
        WaypointSE2 d = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d()),
                new DirectionSE2(0, 1, 0), 1);
        WaypointSE2 e = new WaypointSE2(
                new Pose2d(new Translation2d(0, 1), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 f = new WaypointSE2(
                new Pose2d(new Translation2d(2, 1), new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 g = new WaypointSE2(
                new Pose2d(new Translation2d(2, 0), new Rotation2d()),
                new DirectionSE2(-1, 0, 0), 1);

        List<SplineSE2> splines = new ArrayList<>();
        splines.add(new SplineSE2(d, e));
        splines.add(new SplineSE2(e, f));
        splines.add(new SplineSE2(f, g));

        List<VectorSeries> series = new SplineSE2ToVectorSeries(0.1).convert(splines);
        List<VectorSeries> series2 = new SplineSE2ToVectorSeries(0.05).curvature(splines);
        List<VectorSeries> all = new ArrayList<>();
        all.addAll(series);
        all.addAll(series2);
        ChartUtil.plotOverlay(all, 300);
    }

    @Test
    void testMulti2() {
        WaypointSE2 h = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 i = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 j = new WaypointSE2(
                new Pose2d(new Translation2d(2, 1), new Rotation2d()),
                new DirectionSE2(1, 1, 0), 1.3);
        WaypointSE2 k = new WaypointSE2(
                new Pose2d(new Translation2d(3, 0), new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1.3);
        WaypointSE2 l = new WaypointSE2(
                new Pose2d(new Translation2d(3, -1), new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);

        List<SplineSE2> splines = new ArrayList<>();
        splines.add(new SplineSE2(h, i));
        splines.add(new SplineSE2(i, j));
        splines.add(new SplineSE2(j, k));
        splines.add(new SplineSE2(k, l));

        List<VectorSeries> series = new SplineSE2ToVectorSeries(0.1).convert(splines);
        List<VectorSeries> series2 = new SplineSE2ToVectorSeries(0.05).curvature(splines);
        List<VectorSeries> all = new ArrayList<>();
        all.addAll(series);
        all.addAll(series2);
        ChartUtil.plotOverlay(all, 300);
    }

    @Test
    void testMulti3() {
        WaypointSE2 a = new WaypointSE2(
                new Pose2d(new Translation2d(0, 2), new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 b = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 c = new WaypointSE2(
                new Pose2d(new Translation2d(2, 2), new Rotation2d(Math.PI)),
                new DirectionSE2(0, 1, 0), 1);

        List<SplineSE2> splines = new ArrayList<>();
        splines.add(new SplineSE2(a, b));
        splines.add(new SplineSE2(b, c));

        List<VectorSeries> series = new SplineSE2ToVectorSeries(0.1).convert(splines);
        List<VectorSeries> series2 = new SplineSE2ToVectorSeries(0.1).curvature(splines);
        List<VectorSeries> all = new ArrayList<>();
        all.addAll(series);
        all.addAll(series2);
        ChartUtil.plotOverlay(all, 300);
    }

    @Test
    void testMulti4() {
        WaypointSE2 d = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d()),
                new DirectionSE2(0, 1, 0), 1);
        WaypointSE2 e = new WaypointSE2(
                new Pose2d(new Translation2d(0, 1), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 f = new WaypointSE2(
                new Pose2d(new Translation2d(2, 1), new Rotation2d(Math.PI)),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 g = new WaypointSE2(
                new Pose2d(new Translation2d(2, 0), new Rotation2d()),
                new DirectionSE2(-1, 0, 0), 1);

        List<SplineSE2> splines = new ArrayList<>();
        splines.add(new SplineSE2(d, e));
        splines.add(new SplineSE2(e, f));
        splines.add(new SplineSE2(f, g));

        List<VectorSeries> series = new SplineSE2ToVectorSeries(0.1).convert(splines);
        ChartUtil.plotOverlay(series, 300);
    }

    @Test
    void testMulti5() {
        WaypointSE2 h = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 i = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 j = new WaypointSE2(
                new Pose2d(new Translation2d(2, 1), new Rotation2d(Math.PI)),
                new DirectionSE2(1, 1, 0), 1);
        WaypointSE2 k = new WaypointSE2(
                new Pose2d(new Translation2d(3, 0), new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 l = new WaypointSE2(
                new Pose2d(new Translation2d(3, -1), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(0, -1, 0), 1);

        List<SplineSE2> splines = new ArrayList<>();
        splines.add(new SplineSE2(h, i));
        splines.add(new SplineSE2(i, j));
        splines.add(new SplineSE2(j, k));
        splines.add(new SplineSE2(k, l));

        List<VectorSeries> series = new SplineSE2ToVectorSeries(0.1).convert(splines);
        ChartUtil.plotOverlay(series, 300);

    }

    @Test
    void testCurvature4() {
        // check the sign of the curvature
        List<WaypointSE2> waypoints = Arrays.asList(
                WaypointSE2.irrotational(new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                WaypointSE2.irrotational(new Pose2d(1, 1, new Rotation2d(0)), 0, 1.2));
        List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);
        SplineSE2 spline = splines.get(0);
        // path first turns left then right
        assertTrue(spline.curvature(0.2) > 0, String.format("%f", spline.curvature(0.2)));
        assertTrue(spline.curvature(0.8) < 0, String.format("%f", spline.curvature(0.8)));
    }

    @Test
    void testT() {
        // direction T and spline T are the same
        List<WaypointSE2> waypoints = Arrays.asList(
                new WaypointSE2(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(1, 1, new Rotation2d(2.5)),
                        new DirectionSE2(0, 1, 0), 1.2));
        List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);
        SplineSE2 spline = splines.get(0);
        for (double s = 0; s <= 1.0; s += 0.01) {
            PathSE2Point p = spline.point(s);
            Vector<N2> sT = spline.T(s);
            Vector<N2> pT = p.waypoint().course().T();
            if (DEBUG)
                System.out.printf("%s %s\n", StrUtil.vecStr(sT), StrUtil.vecStr(pT));
            assertTrue(Math100.epsilonEquals(sT, pT), String.format("%f %s %s",
                    s, StrUtil.vecStr(sT), StrUtil.vecStr(pT)));
        }
    }

    @Test
    void testHeadingRate2() {
        List<WaypointSE2> waypoints = Arrays.asList(
                new WaypointSE2(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(1, 1, new Rotation2d(2.5)),
                        new DirectionSE2(0, 1, 0), 1.2));
        List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);
        SplineSE2 spline = splines.get(0);
        for (double s = 0; s <= 1.0; s += 0.01) {
            PathSE2Point p = spline.point(s);
            double sH = spline.headingRate(s);
            double cH = p.waypoint().course().headingRate();
            if (DEBUG)
                System.out.printf("%f %f\n", sH, cH);
            assertTrue(Math100.epsilonEquals(sH, cH),
                    String.format("%f %f %f",
                            s, sH, cH));
        }
    }

}
