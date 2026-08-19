package org.team100.lib.spline.se3;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se3.DirectionSE3;
import org.team100.lib.geometry.se3.WaypointSE3;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.ChartUtil3d;

import org.wpilib.math.linalg.Vector;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.numbers.N3;

public class SplineSE3Test implements Timeless {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    @Test
    void testLinear() {
        WaypointSE3 w0 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(),
                        new Rotation3d()),
                new DirectionSE3(1, 0, 0, 0, 0, 0), 1);
        WaypointSE3 w1 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(1, 0, 0),
                        new Rotation3d()),
                new DirectionSE3(1, 0, 0, 0, 0, 0), 1);
        SplineSE3 spline = new SplineSE3(w0, w1);
        Translation3d t = spline.pose(0).getTranslation();
        assertEquals(0, t.getX(), DELTA);
        t = spline.pose(1).getTranslation();
        assertEquals(1, t.getX(), DELTA);
    }

    @Test
    void testDump() {
        // note rotational direction is from yaw-left to pitch-down-and-yaw-left
        // so the net is pitch-down
        WaypointSE3 w0 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(),
                        new Rotation3d(0, 0, Math.PI / 2)),
                new DirectionSE3(1, 0, 1, 0, 1, 0), 1);
        WaypointSE3 w1 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(1, 1, 1),
                        new Rotation3d(0, Math.PI / 2, Math.PI / 2)),
                new DirectionSE3(0, 1, -1, 0, 1, 0), 1);
        SplineSE3 spline = new SplineSE3(w0, w1);
        ChartUtil3d.plot3dVectorSeries(
                new SplineSE3ToVectorSeries(0.1).convert(List.of(spline)));
        if (DEBUG)
            spline.dump();
    }

    @Test
    void testZeroCurvature() {
        // from (0,0,0) to (1,1,1) in a straight line
        WaypointSE3 w0 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(0, 0, 0),
                        new Rotation3d(0, 0, 0)),
                new DirectionSE3(1, 1, 1, 0, 0, 0), 1);
        WaypointSE3 w1 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(1, 1, 1),
                        new Rotation3d(0, 0, 0)),
                new DirectionSE3(1, 1, 1, 0, 0, 0), 1);
        SplineSE3 spline = new SplineSE3(w0, w1);
        Vector<N3> K = spline.K(0.5);
        assertEquals(0, K.get(0), DELTA);
        assertEquals(0, K.get(1), DELTA);
        assertEquals(0, K.get(2), DELTA);
    }

    @Test
    void testZeroHeadingRate() {
        // from (0,0,0) to (1,1,1) in a straight line
        WaypointSE3 w0 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(0, 0, 0),
                        new Rotation3d(0, 0, 0)),
                new DirectionSE3(1, 1, 1, 0, 0, 0), 1);
        WaypointSE3 w1 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(1, 1, 1),
                        new Rotation3d(0, 0, 0)),
                new DirectionSE3(1, 1, 1, 0, 0, 0), 1);
        SplineSE3 spline = new SplineSE3(w0, w1);
        Vector<N3> H = spline.headingRate(0.5);
        assertEquals(0, H.get(0), DELTA);
        assertEquals(0, H.get(1), DELTA);
        assertEquals(0, H.get(2), DELTA);
    }

    /**
     * https://colab.research.google.com/drive/1iZU72lggE4oH551WXamc-9_Mh_1zR0kV#scrollTo=IFKxOJBoXLEr
     */
    @Test
    void testHelixCurvature() {
        // vector should point roughly in the direction of motion
        WaypointSE3 w0 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(0, 0, 0),
                        new Rotation3d(0, -Math.PI / 4, 0)),
                new DirectionSE3(1, 0, 1, 0, 0, 2), 1);
        WaypointSE3 w1 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(1, 1, 1),
                        new Rotation3d(0, -Math.PI / 4, Math.PI / 2)),
                new DirectionSE3(0, 1, 1, 0, 0, 2), 1);
        SplineSE3 spline = new SplineSE3(w0, w1);
        for (double s = 0; s <= 1; s += 0.1) {
            Vector<N3> K = spline.K(s);
            // Rotation3d r = p.waypoint().pose().getRotation();
            // System.out.printf("R (%5.3f, %5.3f, %5.3f)\n", r.getX(), r.getY(), r.getZ());
            // curvature should be roughly towards (0,1) with z of ~zero.
            if (DEBUG)
                System.out.printf("K = (%5.3f, %5.3f, %5.3f)\n",
                        K.get(0), K.get(1), K.get(2));
        }
        if (DEBUG)
            spline.dump();
    }

    /**
     * https://colab.research.google.com/drive/1iZU72lggE4oH551WXamc-9_Mh_1zR0kV#scrollTo=IFKxOJBoXLEr
     */
    @Test
    void testHelixHeading() {
        // vector should point roughly in the direction of motion
        WaypointSE3 w0 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(0, 0, 0),
                        new Rotation3d(0, -Math.PI / 4, 0)),
                new DirectionSE3(1, 0, 1, 0, 0, 2), 1);
        WaypointSE3 w1 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(1, 1, 1),
                        new Rotation3d(0, -Math.PI / 4, Math.PI / 2)),
                new DirectionSE3(0, 1, 1, 0, 0, 2), 1);
        SplineSE3 spline = new SplineSE3(w0, w1);
        for (double s = 0; s <= 1; s += 0.1) {
            Vector<N3> H = spline.headingRate(s);
            // in the helix, the heading only changes about the z axis
            // and it should be roughly constant.
            // total heading change is 1.5ish, total path length is around 1.9
            // so heading change per meter is around 0.8.
            if (DEBUG)
                System.out.printf("H = (%5.3f, %5.3f, %5.3f)\n",
                        H.get(0), H.get(1), H.get(2));
        }
        if (DEBUG)
            spline.dump();
    }

}
