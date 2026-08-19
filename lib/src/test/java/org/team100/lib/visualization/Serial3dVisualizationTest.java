package org.team100.lib.visualization;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.util.Color;

public class Serial3dVisualizationTest {
    private static final boolean DEBUG = false;

    @Test
    void testFoo() {
        Serial3dVisualization foo = new Serial3dVisualization(() -> List.of());
        // unrotated camera below the table
        Pose3d cameraPose = new Pose3d(0, 0, -4, new Rotation3d(0, 0, 0));
        // a cube centered on the origin
        List<Pose3d> tList = List.of(
                new Pose3d(1, 1, 1, Rotation3d.kZero),
                new Pose3d(-1, 1, 1, Rotation3d.kZero),
                new Pose3d(-1, -1, 1, Rotation3d.kZero),
                new Pose3d(1, -1, 1, Rotation3d.kZero),
                new Pose3d(1, 1, -1, Rotation3d.kZero),
                new Pose3d(-1, 1, -1, Rotation3d.kZero),
                new Pose3d(-1, -1, -1, Rotation3d.kZero),
                new Pose3d(1, -1, -1, Rotation3d.kZero));
        MatOfPoint2f points = foo.project(
                cameraPose, tList, Color.ALICE_BLUE)
                .getFirst();
        List<Point> pointList = points.toList();
        if (DEBUG)
            System.out.printf("pointList.size() = %d\n", pointList.size());
        for (Point p : pointList) {
            if (DEBUG)
                System.out.printf("%5.3f, %5.3f\n", p.x, p.y);
        }

    }

}
