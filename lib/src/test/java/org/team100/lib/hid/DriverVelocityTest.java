package org.team100.lib.hid;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.testing.Timeless;

import org.wpilib.math.geometry.Rotation2d;

public class DriverVelocityTest implements Timeless {
    private static final double DELTA = 0.001;

    @Test
    void testClip0() {
        // zero is no-op
        DriverVelocity input = new DriverVelocity(0, 0, 0);
        DriverVelocity actual = input.clip(1);
        assertEquals(0, actual.x(), DELTA);
        assertEquals(0, actual.y(), DELTA);
        assertEquals(0, actual.theta(), DELTA);
    }

    @Test
    void testClip1() {
        // clip to the unit circle.
        DriverVelocity input = new DriverVelocity(1, 1, 0);
        DriverVelocity actual = input.clip(1);
        assertEquals(0.707, actual.x(), DELTA);
        assertEquals(0.707, actual.y(), DELTA);
    }

    @Test
    void testClip2() {
        // leave the inside alone
        DriverVelocity input = new DriverVelocity(0.5, 0.5, 0);
        DriverVelocity actual = input.clip(1);
        assertEquals(0.5, actual.x(), DELTA);
        assertEquals(0.5, actual.y(), DELTA);
    }

    @Test
    void testDiamond0() {
        // zero is no-op
        DriverVelocity input = new DriverVelocity(0, 0, 0);
        DriverVelocity actual = input.diamond(1, 1, Rotation2d.kZero);
        assertEquals(0, actual.x(), DELTA);
        assertEquals(0, actual.y(), DELTA);
        assertEquals(0, actual.theta(), DELTA);
    }

    @Test
    void testDiamond1() {
        // ahead: get maxX
        DriverVelocity input = new DriverVelocity(1, 0, 0);
        DriverVelocity actual = input.diamond(1, 0.5, Rotation2d.kZero);
        assertEquals(1, actual.x(), DELTA);
        assertEquals(0, actual.y(), DELTA);
    }

    @Test
    void testDiamond1_5() {
        // right, rotated: get maxY
        DriverVelocity input = new DriverVelocity(1, 0, 0);
        DriverVelocity actual = input.diamond(1, 0.5, Rotation2d.kCCW_90deg);
        assertEquals(0.5, actual.x(), DELTA);
        assertEquals(0, actual.y(), DELTA);
    }

    @Test
    void testDiamond2() {
        // left: get maxY
        DriverVelocity input = new DriverVelocity(0, 1, 0);
        DriverVelocity actual = input.diamond(1, 0.5, Rotation2d.kZero);
        assertEquals(0, actual.x(), DELTA);
        assertEquals(0.5, actual.y(), DELTA);

    }

    @Test
    void testSquare() {
        // diagonal: get less
        DriverVelocity input = new DriverVelocity(1, 1, 0);
        DriverVelocity actual = input.diamond(1, 1, Rotation2d.kZero);
        assertEquals(0.5, actual.x(), DELTA);
        assertEquals(0.5, actual.y(), DELTA);
    }

    @Test
    void testDiamond3() {
        // diagonal: get less; preserve angle
        DriverVelocity input = new DriverVelocity(1, 1, 0);
        DriverVelocity actual = input.diamond(1, 0.5, Rotation2d.kZero);
        assertEquals(0.333, actual.x(), DELTA);
        assertEquals(0.333, actual.y(), DELTA);
    }

    @Test
    void testDiamond4() {
        // ahead but with angle
        DriverVelocity input = new DriverVelocity(0, 1, 0);
        DriverVelocity actual = input.diamond(1, 0.5, Rotation2d.kCCW_90deg);
        assertEquals(0, actual.x(), DELTA);
        assertEquals(1, actual.y(), DELTA);
    }

    @Test
    void testDiamond5() {
        // leave the inside alone
        DriverVelocity input = new DriverVelocity(0.5, 0.5, 0);
        DriverVelocity actual = input.diamond(1, 1, Rotation2d.kZero);
        assertEquals(0.5, actual.x(), DELTA);
        assertEquals(0.5, actual.y(), DELTA);
    }

    @Test
    void testsquashedDiamond0() {
        // zero is no-op
        DriverVelocity input = new DriverVelocity(0, 0, 0);
        DriverVelocity actual = input.squashedDiamond(1, 1, Rotation2d.kZero);
        assertEquals(0, actual.x(), DELTA);
        assertEquals(0, actual.y(), DELTA);
        assertEquals(0, actual.theta(), DELTA);
    }

    @Test
    void testsquashedDiamond1() {
        // ahead: get maxX
        DriverVelocity input = new DriverVelocity(1, 0, 0);
        DriverVelocity actual = input.squashedDiamond(1, 0.5, Rotation2d.kZero);
        assertEquals(1, actual.x(), DELTA);
        assertEquals(0, actual.y(), DELTA);
    }

    @Test
    void testsquashedDiamond1_5() {
        // right, rotated: get maxY
        DriverVelocity input = new DriverVelocity(1, 0, 0);
        DriverVelocity actual = input.squashedDiamond(1, 0.5, Rotation2d.kCCW_90deg);
        assertEquals(0.5, actual.x(), DELTA);
        assertEquals(0, actual.y(), DELTA);
    }

    @Test
    void testsquashedDiamond2() {
        // left: get maxY
        DriverVelocity input = new DriverVelocity(0, 1, 0);
        DriverVelocity actual = input.squashedDiamond(1, 0.5, Rotation2d.kZero);
        assertEquals(0, actual.x(), DELTA);
        assertEquals(0.5, actual.y(), DELTA);

    }

    @Test
    void testsquashedSquare() {
        //
        DriverVelocity input = new DriverVelocity(0.707, 0.707, 0);
        DriverVelocity actual = input.squashedDiamond(1, 1, Rotation2d.kZero);
        assertEquals(0.5, actual.x(), DELTA);
        assertEquals(0.5, actual.y(), DELTA);
    }

    @Test
    void testsquashedDiamond3() {
        // diagonal: get less; preserve angle
        DriverVelocity input = new DriverVelocity(0.707, 0.707, 0);
        DriverVelocity actual = input.squashedDiamond(1, 0.5, Rotation2d.kZero);
        assertEquals(0.333, actual.x(), DELTA);
        assertEquals(0.333, actual.y(), DELTA);
    }

    @Test
    void testsquashedDiamond4() {
        // ahead but with angle
        DriverVelocity input = new DriverVelocity(0, 1, 0);
        DriverVelocity actual = input.squashedDiamond(1, 0.5, Rotation2d.kCCW_90deg);
        assertEquals(0, actual.x(), DELTA);
        assertEquals(1, actual.y(), DELTA);
    }

    @Test
    void testsquashedDiamond5() {
        // inside is also scaled.  this is half max input
        DriverVelocity input = new DriverVelocity(0.353, 0.353, 0);
        DriverVelocity actual = input.squashedDiamond(1, 1, Rotation2d.kZero);
        assertEquals(0.25, actual.x(), DELTA);
        assertEquals(0.25, actual.y(), DELTA);
    }

}
