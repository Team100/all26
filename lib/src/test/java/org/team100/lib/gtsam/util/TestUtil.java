package org.team100.lib.gtsam.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import gtsam.Matrix;
import gtsam.Pose2;
import gtsam.Rot2;
import gtsam.Vector;

public class TestUtil {
    public static void assertAllClose(Vector a, Vector b, double tol) throws Throwable {
        int aRows = a.rows();
        int bRows = b.rows();
        assertEquals(aRows, bRows);
        for (int r = 0; r < aRows; ++r) {
            assertEquals(a.at(r), b.at(r), tol);
        }
    }

    public static void assertAllClose(Matrix a, Matrix b, double tol) throws Throwable {
        int aRows = a.rows();
        int bRows = b.rows();
        assertEquals(aRows, bRows);
        int aCols = a.cols();
        int bCols = b.cols();
        assertEquals(aCols, bCols);
        for (int r = 0; r < aRows; ++r) {
            for (int c = 0; c < aCols; ++c) {
                assertEquals(a.at(r, c), b.at(r, c), tol);
            }
        }
    }

    public static void assertPose2Equals(Pose2 a, Pose2 b, double tol) throws Throwable {
        assertEquals(a.x(), b.x(), tol);
        assertEquals(a.y(), b.y(), tol);
        Rot2 ar = a.r();
        Rot2 br = b.r();
        assertEquals(ar.c(), br.c(), tol);
        assertEquals(ar.s(), br.s(), tol);
    }
}
