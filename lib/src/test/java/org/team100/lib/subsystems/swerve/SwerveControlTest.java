package org.team100.lib.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Point;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveControlTest {
    private static final double DELTA = 0.001;

    @Test
    void testTransform() {
        Pose2d p = new Pose2d(new Translation2d(1, 1), new Rotation2d(1));
        VelocitySE2 t = new VelocitySE2(1, 1, 1);
        ControlSE2 s = new ControlSE2(p, t);
        assertEquals(1, s.x().x(), DELTA);
    }

    /** Motionless */
    @Test
    void testTimedState() {
        PathSE2Point p = new PathSE2Point(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                VecBuilder.fill(0, 0));
        TrajectorySE2Point pp = new TrajectorySE2Point(p, 0, 0, 0);
        ControlSE2 s = pp.control();
        assertEquals(0, s.x().x(), DELTA);
        assertEquals(0, s.x().v(), DELTA);
        assertEquals(0, s.x().a(), DELTA);
        assertEquals(0, s.y().x(), DELTA);
        assertEquals(0, s.y().v(), DELTA);
        assertEquals(0, s.y().a(), DELTA);
    }

    /** Accelerating */
    @Test
    void testTimedState2() {
        PathSE2Point p = new PathSE2Point(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                VecBuilder.fill(0, 0));
        TrajectorySE2Point pp = new TrajectorySE2Point(p, 0, 0, 1);
        ControlSE2 s = pp.control();
        assertEquals(0, s.x().x(), DELTA);
        assertEquals(0, s.x().v(), DELTA);
        assertEquals(1, s.x().a(), DELTA);
        assertEquals(0, s.y().x(), DELTA);
        assertEquals(0, s.y().v(), DELTA);
        assertEquals(0, s.y().a(), DELTA);
    }

    /** Constant velocity */
    @Test
    void testTimedState3() {
        PathSE2Point p = new PathSE2Point(
                WaypointSE2.irrotational(
                        new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                VecBuilder.fill(0, 0));
        TrajectorySE2Point pp = new TrajectorySE2Point(p, 0, 1, 0);
        ControlSE2 s = pp.control();
        assertEquals(0, s.x().x(), DELTA);
        assertEquals(1, s.x().v(), DELTA);
        assertEquals(0, s.x().a(), DELTA);
        assertEquals(0, s.y().x(), DELTA);
        assertEquals(0, s.y().v(), DELTA);
        assertEquals(0, s.y().a(), DELTA);
    }

    /** Centrifugal force: +x motion, positive curvature => +y accel. */
    @Test
    void testTimedState4() {
        PathSE2Point p = new PathSE2Point(
                WaypointSE2.irrotational(new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                VecBuilder.fill(0, 1));
        TrajectorySE2Point pp = new TrajectorySE2Point(p, 0, 1, 0);
        ControlSE2 s = pp.control();
        assertEquals(0, s.x().x(), DELTA);
        assertEquals(1, s.x().v(), DELTA);
        assertEquals(0, s.x().a(), DELTA);
        assertEquals(0, s.y().x(), DELTA);
        assertEquals(0, s.y().v(), DELTA);
        assertEquals(1, s.y().a(), DELTA);
    }

    @Test
    void testChassisSpeeds0() {
        ControlSE2 state = new ControlSE2(
                new Pose2d(new Translation2d(0, 0), Rotation2d.kPi),
                new VelocitySE2(1, 0, 0));
        ChassisSpeeds speeds = state.chassisSpeeds();
        assertEquals(-1, speeds.vxMetersPerSecond, DELTA);
        assertEquals(0, speeds.vyMetersPerSecond, DELTA);
        assertEquals(0, speeds.omegaRadiansPerSecond, DELTA);
    }

    @Test
    void testChassisSpeeds1() {
        ControlSE2 state = new ControlSE2(
                new Pose2d(new Translation2d(0, 0), Rotation2d.kCCW_Pi_2),
                new VelocitySE2(1, 0, 1));
        ChassisSpeeds speeds = state.chassisSpeeds();
        assertEquals(0, speeds.vxMetersPerSecond, DELTA);
        assertEquals(-1, speeds.vyMetersPerSecond, DELTA);
        assertEquals(1, speeds.omegaRadiansPerSecond, DELTA);
    }
}
