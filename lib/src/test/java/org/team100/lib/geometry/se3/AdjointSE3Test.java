package org.team100.lib.geometry.se3;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.testing.TestUtil;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Twist3d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N6;

public class AdjointSE3Test {

    @Test
    void test0() {
        Pose3d p = Pose3d.kZero;
        Matrix<N6, N6> ad = AdjointSE3.ad(p);
        // twist +x in body frame
        Twist3d t = new Twist3d(1, 0, 0, 0, 0, 0);
        Vector<N6> v = GeometryUtil.toVec(t);
        Matrix<N6, N1> v1 = ad.times(v);
        // identity pose -> no change in twist
        TestUtil.verify(new Twist3d(1, 0, 0, 0, 0, 0), v1);
        Matrix<N6, N6> adInv = AdjointSE3.adInv(p);
        Matrix<N6, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

    @Test
    void test1() {
        Pose3d p = new Pose3d(1, 0, 0, new Rotation3d());
        Matrix<N6, N6> ad = AdjointSE3.ad(p);
        Twist3d t = new Twist3d(1, 0, 0, 0, 0, 0);
        Vector<N6> v = GeometryUtil.toVec(t);
        Matrix<N6, N1> v1 = ad.times(v);
        // no rotation -> no change in twist translation
        TestUtil.verify(new Twist3d(1, 0, 0, 0, 0, 0), v1);
        Matrix<N6, N6> adInv = AdjointSE3.adInv(p);
        Matrix<N6, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

    @Test
    void test2() {
        Pose3d p = new Pose3d(0, 1, 0, new Rotation3d());
        Matrix<N6, N6> ad = AdjointSE3.ad(p);
        Twist3d t = new Twist3d(1, 0, 0, 0, 0, 0);
        Vector<N6> v = GeometryUtil.toVec(t);
        Matrix<N6, N1> v1 = ad.times(v);
        // no rotation -> no change in twist translation
        TestUtil.verify(new Twist3d(1, 0, 0, 0, 0, 0), v1);
        Matrix<N6, N6> adInv = AdjointSE3.adInv(p);
        Matrix<N6, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

    @Test
    void test3() {
        Pose3d p = new Pose3d(0, 0, 0, new Rotation3d(0, 0, Math.PI / 2));
        Matrix<N6, N6> ad = AdjointSE3.ad(p);
        Twist3d t = new Twist3d(1, 0, 0, 0, 0, 0);
        Vector<N6> v = GeometryUtil.toVec(t);
        Matrix<N6, N1> v1 = ad.times(v);
        // pose rotation transforms twist translation
        TestUtil.verify(new Twist3d(0, 1, 0, 0, 0, 0), v1);
        Matrix<N6, N6> adInv = AdjointSE3.adInv(p);
        Matrix<N6, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

    @Test
    void test4() {
        Pose3d p = new Pose3d(1, 0, 0, new Rotation3d(0, 0, Math.PI / 2));
        Matrix<N6, N6> ad = AdjointSE3.ad(p);
        Twist3d t = new Twist3d(1, 0, 0, 0, 0, 0);
        Vector<N6> v = GeometryUtil.toVec(t);
        Matrix<N6, N1> v1 = ad.times(v);
        // pose rotation transforms twist translation
        TestUtil.verify(new Twist3d(0, 1, 0, 0, 0, 0), v1);
        Matrix<N6, N6> adInv = AdjointSE3.adInv(p);
        Matrix<N6, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

    @Test
    void test5() {
        Pose3d p = new Pose3d(1, 0, 0, new Rotation3d(0, 0, 0));
        Matrix<N6, N6> ad = AdjointSE3.ad(p);
        Twist3d t = new Twist3d(0, 0, 0, 0, 0, 1);
        Vector<N6> v = GeometryUtil.toVec(t);
        Matrix<N6, N1> v1 = ad.times(v);
        // origin has to move -y to keep rotational center still
        TestUtil.verify(new Twist3d(0, -1, 0, 0, 0, 1), v1);
        Matrix<N6, N6> adInv = AdjointSE3.adInv(p);
        Matrix<N6, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

    @Test
    void test6() {
        Pose3d p = new Pose3d(1, 0, 0, new Rotation3d(0, 0, Math.PI / 2));
        Matrix<N6, N6> ad = AdjointSE3.ad(p);
        Twist3d t = new Twist3d(0, 0, 0, 0, 0, 1);
        Vector<N6> v = GeometryUtil.toVec(t);
        Matrix<N6, N1> v1 = ad.times(v);
        // parallel rotation does not matter
        TestUtil.verify(new Twist3d(0, -1, 0, 0, 0, 1), v1);
        Matrix<N6, N6> adInv = AdjointSE3.adInv(p);
        Matrix<N6, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }

    @Test
    void test7() {
        Pose3d p = new Pose3d(1, 0, 0, new Rotation3d(0, 0, Math.PI / 2));
        Matrix<N6, N6> ad = AdjointSE3.ad(p);
        Twist3d t = new Twist3d(0, 0, 0, 0, 1, 0);
        Vector<N6> v = GeometryUtil.toVec(t);
        Matrix<N6, N1> v1 = ad.times(v);
        // pose rotation transforms twist rotation
        TestUtil.verify(new Twist3d(0, 0, 0, -1, 0, 0), v1);
        Matrix<N6, N6> adInv = AdjointSE3.adInv(p);
        Matrix<N6, N1> v2 = adInv.times(v1);
        TestUtil.verify(t, v2);
    }
}
