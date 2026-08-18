package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class Math100Test {
    private static final double DELTA = 0.001;

    @Test
    void testEpsilonEquals() {
        assertTrue(Math100.epsilonEquals(1, 1));
        assertFalse(Math100.epsilonEquals(1, 1.01));
    }

    @Test
    void testSolveQuadraticTwo() {
        List<Double> roots = Math100.solveQuadratic(1, 0, -1);
        assertEquals(2, roots.size());
        assertEquals(1, roots.get(0), DELTA);
        assertEquals(-1, roots.get(1), DELTA);
    }

    @Test
    void testSolveQuadraticOne() {
        List<Double> roots = Math100.solveQuadratic(1, 2, 1);
        assertEquals(1, roots.size());
        assertEquals(-1, roots.get(0), DELTA);
    }

    @Test
    void testSolveQuadraticZero() {
        List<Double> roots = Math100.solveQuadratic(1, 0, 1);
        assertEquals(0, roots.size());
    }

    @Test
    void testGetMinDistance() {
        double measurement = 4;
        double x = 0;
        double d = Math100.getMinDistance(measurement, x);
        assertEquals(2 * Math.PI, d, DELTA);
    }

    @Test
    void testAccel() {
        // average v = 0.5
        // dv = 1
        assertEquals(0.5, Math100.accel(0, 1, 1.0), 0.001);
        assertEquals(1.0, Math100.accel(0, 1, 0.5), 0.001);
        // average v = 1.5
        // dv = 1
        assertEquals(1.5, Math100.accel(1, 2, 1.0), 0.001);
        // same case, backwards
        assertEquals(1.5, Math100.accel(2, 1, -1.0), 0.001);
    }

    /**
     * Fundamental math.
     * 
     * Components are treated as independent.
     */
    @Test
    void testBasic0() {
        // Given two states, find the acceleration between them.
        ModelSE2 s0 = new ModelSE2(
                new Pose2d(0, 0, new Rotation2d(0)),
                new VelocitySE2(0, 0, 0));
        ModelSE2 s1 = new ModelSE2(
                new Pose2d(1, 0, new Rotation2d(0)),
                new VelocitySE2(1, 0, 0));
        AccelerationSE2 a = new AccelerationSE2(
                Math100.accel(
                        s0.velocity().x(), s1.velocity().x(), s1.pose().getX() - s0.pose().getX()),
                Math100.accel(
                        s0.velocity().y(), s1.velocity().y(), s1.pose().getY() - s0.pose().getY()),
                Math100.accel(
                        s0.velocity().theta(), s1.velocity().theta(),
                        s1.pose().getRotation().minus(s0.pose().getRotation()).getRadians()));
        // 1 meter at 1 m/s, a=0.5 m/s, t= 2
        assertEquals(0.5, a.x(), DELTA);
        assertEquals(0, a.y(), DELTA);
        assertEquals(0, a.theta(), DELTA);
    }

    @Test
    void testBasic1() {
        // This case makes no sense.
        ModelSE2 s0 = new ModelSE2(
                new Pose2d(0, 0, new Rotation2d(0)),
                new VelocitySE2(0, 0, 0));
        ModelSE2 s1 = new ModelSE2(
                new Pose2d(1, 0, new Rotation2d(0)), // <<< positive position
                new VelocitySE2(-1, 0, 0)); // <<< negative velocity
        AccelerationSE2 a = new AccelerationSE2(
                Math100.accel(
                        s0.velocity().x(), s1.velocity().x(), s1.pose().getX() - s0.pose().getX()),
                Math100.accel(
                        s0.velocity().y(), s1.velocity().y(), s1.pose().getY() - s0.pose().getY()),
                Math100.accel(
                        s0.velocity().theta(), s1.velocity().theta(),
                        s1.pose().getRotation().minus(s0.pose().getRotation()).getRadians()));
        // 1 meter at 1 m/s, a=0.5 m/s, t= 2
        // this acceleration is for negative time, i.e. from s1 to s0, decelerating.
        assertEquals(0.5, a.x(), DELTA);
        assertEquals(0, a.y(), DELTA);
        assertEquals(0, a.theta(), DELTA);
    }

    @Test
    void testV1() {
        // no v or a => no new v
        assertEquals(0.0, Math100.v1(0, 0, 1.0));
        // no a => keep old v
        assertEquals(1.0, Math100.v1(1, 0, 1.0));
        // a = 0.5 for 1 => final v is 1
        assertEquals(1.0, Math100.v1(0, 0.5, 1.0));
        // same case, backwards
        assertEquals(0.0, Math100.v1(1.0, 0.5, -1.0));
        // backwards with negative accel
        assertEquals(1.0, Math100.v1(0.0, -0.5, -1.0));
    }

}
