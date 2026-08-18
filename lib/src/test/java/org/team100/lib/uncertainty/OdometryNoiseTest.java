package org.team100.lib.uncertainty;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DeltaSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class OdometryNoiseTest {
    private static final double DELTA = 0.001;

    @Test
    void testOdometryCartesian() {
        double odo = OdometryNoise.get(0, 0).cartesian();
        assertEquals(0.0, odo, 1e-6);
        // moving pretty slowly, 0.5 m/s over 0.02 sec
        odo = OdometryNoise.get(0.01, 0).cartesian();
        // about 3 cm/s of error, which seems reasonable
        assertEquals(0.00055, odo, 1e-6);
        // moving pretty fast, 5 m/s over 0.02 sec
        odo = OdometryNoise.get(0.1, 0).cartesian();
        // 0.5 m/s of error, a whole lot!
        assertEquals(0.010, odo, 1e-6);
    }

    @Test
    void testOdometryRotation() {
        double odo = OdometryNoise.get(0, 0).rotation();
        assertEquals(0.0, odo, 1e-6);
        // moving pretty slowly, 0.5 m/s over 0.02 sec
        odo = OdometryNoise.get(0.01, 0).rotation();
        // about 3 cm/s of error, which seems reasonable
        // assertEquals(0.00055, odo, 1e-6);
        assertEquals(0.00255, odo, 1e-6);
        // moving pretty fast, 5 m/s over 0.02 sec
        odo = OdometryNoise.get(0.1, 0).rotation();
        // 0.5 m/s of error, a whole lot!
        // assertEquals(0.010, odo, 1e-6);
        assertEquals(0.030, odo, 1e-6);
        // this is slow rotation, 1 rad/s over 0.02 sec
        odo = OdometryNoise.get(0, 0.02).rotation();
        // 0.06 rad/s of error, a few degrees
        // assertEquals(0.0012, odo, 1e-3);
        assertEquals(0.0052, odo, 1e-3);
        // this is very fast rotation, 10 rad/s over 0.02 sec
        odo = OdometryNoise.get(0, 0.2).rotation();
        // 1.5 rad/s of error, that's a whole lot!
        // assertEquals(0.0300, odo, 1e-3);
        assertEquals(0.0700, odo, 1e-3);
    }

    @Test
    void testTwistVsDelta() {
        // we're using an SE(2) twist to represent the difference between the state and
        // the measurement, and then we scale that twist. But I think that might not be
        // the right thing.
        Pose2d state = new Pose2d();
        Pose2d measurement = new Pose2d(1, 0, new Rotation2d(1));
        Twist2d twist = state.log(measurement);
        double scale = 0.5;
        Twist2d scaledTwist = new Twist2d(scale * twist.dx, scale * twist.dy, scale * twist.dtheta);
        Pose2d result = state.exp(scaledTwist);
        // I would expect the scaling to affect each dimension separately, but that's
        // not what happens.
        assertEquals(0.5000, result.getX(), DELTA);
        assertEquals(-0.128, result.getY(), DELTA);
        assertEquals(0.5000, result.getRotation().getRadians(), DELTA);
        DeltaSE2 delta = DeltaSE2.delta(state, measurement);
        DeltaSE2 scaledDelta = delta.times(scale);
        Pose2d result2 = scaledDelta.plus(state);
        // The delta just applies each dimension separately.
        assertEquals(0.5, result2.getX(), DELTA);
        assertEquals(0.0, result2.getY(), DELTA);
        assertEquals(0.5, result2.getRotation().getRadians(), DELTA);
    }
}
