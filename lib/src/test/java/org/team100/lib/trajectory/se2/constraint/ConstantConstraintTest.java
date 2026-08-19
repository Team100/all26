package org.team100.lib.trajectory.se2.constraint;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.testing.Timeless;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class ConstantConstraintTest implements Timeless {
    private static final double DELTA = 0.001;

    @Test
    void testVelocity() {
        ConstantConstraint c = new ConstantConstraint(2, 3);
        PathSE2Point state = new PathSE2Point(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                VecBuilder.fill(0, 0));
        assertEquals(2, c.maxV(state), DELTA);
    }

    @Test
    void testAccel() {
        ConstantConstraint c = new ConstantConstraint(2, 3);
        PathSE2Point state = new PathSE2Point(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                 VecBuilder.fill(0, 0));
        assertEquals(-3, c.maxDecel(state, 1), DELTA);
        assertEquals(3, c.maxAccel(state, 1), DELTA);

    }

}
