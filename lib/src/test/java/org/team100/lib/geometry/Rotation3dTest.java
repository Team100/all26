package org.team100.lib.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;

public class Rotation3dTest {
    private static final boolean DEBUG = false;

    private Rotation3d lerp(Rotation3d a, Rotation3d b, double t) {
        Quaternion dq = a.getQuaternion().log(b.getQuaternion());
        return new Rotation3d(a.getQuaternion().exp(dq.times(t)));
    }

    @Test
    void testQuaternion() {
        Rotation3d a = new Rotation3d(1, 0, 0);
        Rotation3d b = new Rotation3d(0, 1, 0);
        Rotation3d i = lerp(a, b, 1);
        assertEquals(b, i);
    }

    @Test
    void testInterpolation() {
        Rotation3d a = new Rotation3d(1, 0, 0);
        Rotation3d b = new Rotation3d(0, 1, 0);
        // see https://github.com/wpilibsuite/allwpilib/issues/8523
        assertEquals(b, a.interpolate(b, 1));
    }

    @Test
    void testInterpolation2() {
        Rotation3d a = new Rotation3d(1, 0, 0);
        Rotation3d b = new Rotation3d(0, 1, 0);
        Rotation3d d = b.minus(a);
        // rotation is not commutative.
        // see https://github.com/wpilibsuite/allwpilib/issues/8523
        assertNotEquals(b, a.plus(d));
    }

    @Test
    void testInterpolation3() {
        Rotation3d a = new Rotation3d(1, 0, 0);
        Rotation3d b = new Rotation3d(0, 1, 0);
        Rotation3d d = new Rotation3d(-1, 1, 0);
        assertEquals(b, a.plus(d));
    }

    @Test
    void testInterpolation4() {
        Rotation3d a = new Rotation3d(1, 0, 0);
        Rotation3d b = new Rotation3d(0, 1, 0);
        Quaternion bq = b.getQuaternion();
        if (DEBUG)
            System.out.printf("b (%6.3f %6.3f %6.3f %6.3f)\n", bq.getW(), bq.getX(), bq.getY(), bq.getZ());
        Rotation3d i = a.interpolate(b, 1);
        Quaternion iq = i.getQuaternion();
        if (DEBUG)
            System.out.printf("i (%6.3f %6.3f %6.3f %6.3f)\n", iq.getW(), iq.getX(), iq.getY(), iq.getZ());
    }

}
