package org.team100.lib.trajectory.se2.constraint;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class YawRateConstraintTest implements Timeless {
    private static final double DELTA = 0.001;
    // for testing, use the aboslute maximum. This shouldn't be used in a real
    // robot.
    private static final double YAW_RATE_SCALE = 1.0;

    @Test
    void testNormal() {
        // towards +x, 1 rad/m, 1 m/s wheel -> 1 rad/s limit => 2.8 m/s (which violates
        // the linear constraint but it's ok)
        YawRateConstraint c = new YawRateConstraint(SwerveKinodynamicsFactory.forTest(),
                YAW_RATE_SCALE);
        PathSE2Point p = new PathSE2Point(
                new WaypointSE2(new Pose2d(0, 0, new Rotation2d(0)),
                        new DirectionSE2(1, 0, 1), 1.2),
                VecBuilder.fill(0, 0));
        assertEquals(-8.485, c.maxDecel(p, 0), DELTA);
        assertEquals(8.485, c.maxAccel(p, 0), DELTA);
        assertEquals(2.828, c.maxV(p), DELTA);
    }

    @Test
    void testVelocity2() {
        // towards +x, 1 rad/m, 2 rad/s limit => 2 m/s
        YawRateConstraint c = new YawRateConstraint(SwerveKinodynamicsFactory.forTest2(),
                YAW_RATE_SCALE);
        PathSE2Point p = new PathSE2Point(
                new WaypointSE2(new Pose2d(0, 0, new Rotation2d(0)),
                        new DirectionSE2(1, 0, 1), 1.2),
                VecBuilder.fill(0, 0));
        assertEquals(5.656, c.maxV(p), DELTA);
    }

    @Test
    void testAccel() {
        // we should impose an accel limit, now that the trajectory builder doesn't
        // force omega to zero at the start.
        YawRateConstraint c = new YawRateConstraint(SwerveKinodynamicsFactory.forTest(),
                YAW_RATE_SCALE);
        // driving and spinning
        PathSE2Point p = new PathSE2Point(
                new WaypointSE2(new Pose2d(0, 0, new Rotation2d(0)),
                        new DirectionSE2(1, 0, 1), 1.2),
                VecBuilder.fill(0, 0));
        // there is an accel limit.
        assertEquals(-8.485, c.maxDecel(p, 0), DELTA);
        assertEquals(8.485,
                c.maxAccel(p, 0), DELTA);
    }

    @Test
    void testAccel2() {
        // towards +x, 1 rad/m, 2 rad/s limit => 2 m/s
        double scale = 0.1;
        YawRateConstraint c = new YawRateConstraint(SwerveKinodynamicsFactory.forRealisticTest(),
                scale);
        PathSE2Point p = new PathSE2Point(
                new WaypointSE2(new Pose2d(0, 0, new Rotation2d(0)),
                        new DirectionSE2(1, 0, 1), 1.2),
                VecBuilder.fill(0, 0));
        // this number is still quite high even with a low scale.
        assertEquals(-16.971, c.maxDecel(p, 0), DELTA);
        assertEquals(16.971,
                c.maxAccel(p, 0), DELTA);
    }
}
