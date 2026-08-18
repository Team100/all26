package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class InterceptTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testBothStationary() {
        // robot at the origin, stationary
        // target direction +x, stationary
        Intercept intercept = new Intercept(log);
        Optional<Rotation2d> azimuth = intercept.intercept(
                new Translation2d(0, 0),
                new VelocityR2(0, 0),
                new Translation2d(1, 0),
                new VelocityR2(0, 0),
                1);
        assertTrue(azimuth.isPresent());
        assertEquals(0, azimuth.get().getRadians(), DELTA);
    }

    @Test
    void testRobotStationaryTargetMoving() {
        // robot at the origin, stationary
        // target direction +x,-y, moving +y
        // projectile at 1 m/s, target at 1 m/s,
        // should intercept at y = 0
        Intercept intercept = new Intercept(log);
        Optional<Rotation2d> azimuth = intercept.intercept(
                new Translation2d(0, 0),
                new VelocityR2(0, 0),
                new Translation2d(1, -1),
                new VelocityR2(0, 1),
                1);
        assertTrue(azimuth.isEmpty());
        // assertEquals(0, azimuth.get().getRadians(), DELTA);
    }

    @Test
    void testRobotMovingTargetStationary() {
        // robot at the origin, moving +y
        // target direction +x,+y, stationary
        // projectile at 1 m/s in x, with robot at 1 m/s,
        // should intercept at y = 1
        Intercept intercept = new Intercept(log);
        Optional<Rotation2d> azimuth = intercept.intercept(
                new Translation2d(0, 0),
                new VelocityR2(0, 1),
                new Translation2d(1, 1),
                new VelocityR2(0, 0),
                1);
        assertTrue(azimuth.isEmpty());
        // assertEquals(0, azimuth.get().getRadians(), DELTA);
    }

    @Test
    void testTargetRecedingTooFast() {
        // robot at the origin, stationary
        // target direction +x, moving fast +x
        // projectile at 1 m/s in x, can't catch it.
        Intercept intercept = new Intercept(log);
        Optional<Rotation2d> azimuth = intercept.intercept(
                new Translation2d(0, 0),
                new VelocityR2(0, 0),
                new Translation2d(1, 0),
                new VelocityR2(2, 0),
                1);
        assertTrue(azimuth.isEmpty());
    }

    @Test
    void testTargetCrossingPath() {
        // R0G: Robot at origin
        var robotPos = new Translation2d(0, 0);
        var robotVel = new VelocityR2(0, 0);

        // T0G: Target at (10, 5)
        var targetPos = new Translation2d(10, 5);
        // vTG: Target moving straight across the X-axis at (0, -1) m/s
        var targetVel = new VelocityR2(0, -1);

        // Muzzle speed is fast enough
        double muzzleSpeed = 10.0;

        // vT = (0, -1). A = 1^2 - 10^2 = -99. (A != 0)
        // B = 2(10*0 + 5*-1) = -10.
        // C = 10^2 + 5^2 = 125.
        // Equation: -99t^2 - 10t + 125 = 0.
        // Solution t ~ 1.077s.
        // Interception point I ≈ (10, 3.923).
        // Azimuth should be atan2(3.923, 10) ≈ 0.373 radians.

        double expectedTheta = 0.374;

        Intercept intercept = new Intercept(log);
        Optional<Rotation2d> azimuth = intercept.intercept(
                robotPos,
                robotVel,
                targetPos,
                targetVel,
                muzzleSpeed);

        assertTrue(azimuth.isPresent());
        assertEquals(expectedTheta, azimuth.get().getRadians(), DELTA);
    }
}