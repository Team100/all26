package org.team100.lib.geometry.se2;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.testing.TestUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

public class AdjointSE2Test {
    @Test
    void test0() {
        Pose2d p = Pose2d.kZero;
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        Twist2d t = new Twist2d(1, 0, 0);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        TestUtil.verify(new Twist2d(1, 0, 0), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

    @Test
    void test1() {
        Pose2d p = new Pose2d(1, 0, new Rotation2d());
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        Twist2d t = new Twist2d(1, 0, 0);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        TestUtil.verify(new Twist2d(1, 0, 0), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

    @Test
    void test2() {
        Pose2d p = new Pose2d(0, 1, new Rotation2d());
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        Twist2d t = new Twist2d(1, 0, 0);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        TestUtil.verify(new Twist2d(1, 0, 0), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

    @Test
    void test3() {
        Pose2d p = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        Twist2d t = new Twist2d(1, 0, 0);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        TestUtil.verify(new Twist2d(0, 1, 0), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

    @Test
    void test4() {
        Pose2d p = new Pose2d(1, 0, new Rotation2d(Math.PI / 2));
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        Twist2d t = new Twist2d(1, 0, 0);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        TestUtil.verify(new Twist2d(0, 1, 0), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

    @Test
    void test5() {
        Pose2d p = new Pose2d(1, 0, new Rotation2d());
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        System.out.println(ad);
        Twist2d t = new Twist2d(0, 0, 1);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        // origin has to move -y to keep rotational center still
        TestUtil.verify(new Twist2d(0, -1, 1), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

    @Test
    void test6() {
        Pose2d p = new Pose2d(1, 0, new Rotation2d(Math.PI / 2));
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        Twist2d t = new Twist2d(0, 0, 1);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        // origin has to move -y to keep rotational center still
        TestUtil.verify(new Twist2d(0, -1, 1), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

}
