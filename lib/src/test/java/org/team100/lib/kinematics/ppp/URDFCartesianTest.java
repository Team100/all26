package org.team100.lib.kinematics.ppp;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.Map;

import org.junit.jupiter.api.Test;
import org.team100.lib.testing.TestUtil;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.numbers.N3;

public class URDFCartesianTest {
    private static final boolean DEBUG = false;

    @Test
    void testFwd() {
        URDFCartesian m = URDFCartesian.make();
        Map<String, Double> q = Map.of(
                "base_gantry", 0.0,
                "gantry_head", 0.0,
                "head_spindle", 0.0);
        Map<String, Pose3d> poses = m.forward(q);
        assertEquals(4, poses.size());
        TestUtil.verify(new Pose3d(0, 0, 0.0, new Rotation3d()), poses, "base_gantry");
        TestUtil.verify(new Pose3d(0, 0, 0.0, new Rotation3d()), poses, "gantry_head");
        TestUtil.verify(new Pose3d(0, 0, 0.0, new Rotation3d()), poses, "head_spindle");
        TestUtil.verify(new Pose3d(0, 0, -0.1, new Rotation3d()), poses, "center_point");
    }

    @Test
    void testInv() {
        URDFCartesian m = URDFCartesian.make();
        // desired end effector position is in the center of the
        // [0,1]x[0,1]x[0,0.1] workspace. Since the z extends -0.1
        // down, the actual z will be 0.15.
        Pose3d end = new Pose3d(0.5, 0.5, 0.05, new Rotation3d());
        // initial position is just somewhere random
        Vector<N3> q0 = VecBuilder.fill(0.1, 0.1, 0.1);
        Map<String, Double> qMap = m.inverse(
                q0, "center_point", end);
        assertEquals(3, qMap.size());
        assertEquals(0.5, qMap.get("base_gantry"), 1e-3);
        assertEquals(0.5, qMap.get("gantry_head"), 1e-3);
        assertEquals(0.15, qMap.get("head_spindle"), 1e-3);
    }

    @Test
    void testInvBeyondLimit() {
        URDFCartesian m = URDFCartesian.make();
        // x position is beyond the [0,1] envelope, so it should return 1.
        Pose3d end = new Pose3d(1.5, 0.5, 0.05, new Rotation3d());
        // initial position is just somewhere random
        Vector<N3> q0 = VecBuilder.fill(0.1, 0.1, 0.1);
        // solution is not possible
        assertThrows(IllegalArgumentException.class, () -> m.inverse(
                q0, "center_point", end));
    }

    /** 52 us per solve on my laptop, 2 solver iterations each. */
    @Test
    void testInvPerformance() {
        URDFCartesian m = URDFCartesian.make();
        Pose3d end = new Pose3d(0.5, 0.5, 0.05, new Rotation3d());
        Vector<N3> q0 = VecBuilder.fill(0.1, 0.1, 0.1);
        int iterations = 10000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            m.inverse(q0, "center_point", end);
        }
        long finishTime = System.currentTimeMillis();
        if (DEBUG) {
            System.out.println("Cartesian inverse");
            System.out.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
            System.out.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
        }
    }
}
