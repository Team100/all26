package org.team100.lib.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;

public class Rotation2dTest {

    // this fails
    // see https://github.com/wpilibsuite/allwpilib/issues/8617
    // @Test
    void testInterpolate() {
        Rotation2d x = new Rotation2d(100);
        Rotation2d y = x.interpolate(x, 0);
        assertEquals(x.getRadians(), y.getRadians(), 1e-9);
    }

    // this fails
    // see https://github.com/wpilibsuite/allwpilib/issues/8617
    // @Test
    void testPlus() {
        Rotation2d x = new Rotation2d(100);
        Rotation2d y = x.plus(Rotation2d.kZero);
        assertEquals(x.getRadians(), y.getRadians(), 1e-9);
    }

}
