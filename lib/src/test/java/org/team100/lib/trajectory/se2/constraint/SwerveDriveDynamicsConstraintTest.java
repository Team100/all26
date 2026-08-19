package org.team100.lib.trajectory.se2.constraint;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

class SwerveDriveDynamicsConstraintTest implements Timeless {
    private static final double DELTA = 0.001;

    @Test
    void testVelocity() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        SwerveDriveDynamicsConstraint c = new SwerveDriveDynamicsConstraint(kinodynamics, 1, 1);

        // motionless
        double maxV = c.maxV(new PathSE2Point(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                VecBuilder.fill(0, 0)));
        assertEquals(5, maxV, DELTA);

        // moving in +x, no curvature, no rotation
        maxV = c.maxV(new PathSE2Point(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                VecBuilder.fill(0, 0)));
        // max allowed velocity is full speed
        assertEquals(5, maxV, DELTA);

        // moving in +x, 5 rad/meter
        maxV = c.maxV(new PathSE2Point(
                new WaypointSE2(new Pose2d(0, 0, new Rotation2d(0)),
                        new DirectionSE2(1, 0, 5), 1.2),
                VecBuilder.fill(0, 0)));
        // at 5 rad/m with 0.5m sides the fastest you can go is 1.55 m/s.
        assertEquals(1.925, maxV, DELTA);

        // max wheel speed 5 m/s
        // wheelsbase/track 0.5 m
        // so radius to center is 0.25 * sqrt(2) = 0.356
        // traveling 1 m/s, there are 4 m/s available for the fastest wheel
        // which means 11.314 rad/s, and also 11.314 rad/m since we're going 1 m/s.
        PathSE2Point state = new PathSE2Point(
                new WaypointSE2(new Pose2d(0, 0, new Rotation2d(0)),
                        new DirectionSE2(1, 0, 11.31708), 1.2),
                VecBuilder.fill(0, 0));
        maxV = c.maxV(state);
        // verify corner velocity is full scale
        assertEquals(5, c.maxV());
        // this should be feasible; note it's not exactly 1 due to discretization
        assertEquals(1.036, maxV, DELTA);

    }

    @Test
    void testAccel() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        SwerveDriveDynamicsConstraint c = new SwerveDriveDynamicsConstraint(kinodynamics, 1, 1);
        // this is constant
        Pose2d p = new Pose2d(0, 0, new Rotation2d(0));
        PathSE2Point p2 = new PathSE2Point(
                WaypointSE2.irrotational(p, 0, 1.2), VecBuilder.fill(0, 0));
        assertEquals(-20, c.maxDecel(p2, 0), DELTA);
        assertEquals(10, c.maxAccel(p2, 0), DELTA);
    }
}
