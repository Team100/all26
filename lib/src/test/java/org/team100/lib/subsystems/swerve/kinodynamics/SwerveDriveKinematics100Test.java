package org.team100.lib.subsystems.swerve.kinodynamics;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDelta;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleState100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveDriveKinematics100Test {
    private static final double DELTA = 0.001;

    @Test
    void testCrab() {
        // in this case the wheels are assumed to turn immediately to the new
        // angle, so this is straight back.
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5), // 1,1
                new Translation2d(0.5, -0.5), // 1,0
                new Translation2d(-0.5, 0.5), // 0,1
                new Translation2d(-0.5, -0.5) // origin
        );
        SwerveModulePositions start = new SwerveModulePositions(
                new SwerveModulePosition100(
                        0,
                        Optional.of(Rotation2d.fromRadians(Math.PI / 2))),
                new SwerveModulePosition100(
                        0,
                        Optional.of(Rotation2d.fromRadians(Math.PI / 2))),
                new SwerveModulePosition100(
                        0,
                        Optional.of(Rotation2d.fromRadians(Math.PI / 2))),
                new SwerveModulePosition100(
                        0.0,
                        Optional.of(Rotation2d.fromRadians(Math.PI / 2))));
        SwerveModulePositions end = new SwerveModulePositions(
                new SwerveModulePosition100(
                        1,
                        Optional.of(Rotation2d.fromRadians(Math.PI))),
                new SwerveModulePosition100(
                        1,
                        Optional.of(Rotation2d.fromRadians(Math.PI))),
                new SwerveModulePosition100(
                        1,
                        Optional.of(Rotation2d.fromRadians(Math.PI))),
                new SwerveModulePosition100(
                        1,
                        Optional.of(Rotation2d.fromRadians(Math.PI))));

        SwerveModuleDeltas delta = SwerveModuleDeltas.modulePositionDelta(start, end);

        assertEquals(1, delta.frontLeft().distanceMeters(), DELTA);
        assertEquals(Math.PI, delta.frontLeft().wrappedAngle().get().getRadians(), DELTA);

        Twist2d twist = kinematics.forward(delta);
        assertEquals(-1, twist.dx, DELTA);
        assertEquals(0, twist.dy, DELTA);
        assertEquals(0, twist.dtheta, DELTA);

        // it transforms the starting pose correctly
        Pose2d pStart = new Pose2d(0.5, 0.5, Rotation2d.kZero);
        Pose2d pEnd = pStart.exp(twist);
        assertEquals(-0.5, pEnd.getX(), DELTA);
        assertEquals(0.5, pEnd.getY(), DELTA);
        assertEquals(0, pEnd.getRotation().getRadians(), DELTA);
    }

    @Test
    void testCrabInverse() {
        // inverse kinematics for the above case
        SwerveDriveKinematics100 m_kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));

        Pose2d pStart = new Pose2d(0.5, 0.5, Rotation2d.kZero);
        Pose2d pEnd = new Pose2d(-0.5, 1.5, Rotation2d.kZero);
        Twist2d t = pStart.log(pEnd);
        assertEquals(-1, t.dx, DELTA);
        assertEquals(1, t.dy, DELTA);
        assertEquals(0, t.dtheta, DELTA);

        // the inverse kinematics really just finds the dx and dy for each
        // corner; it doesn't know the path the corners take to get there
        // so it assumes the corner paths are straight lines.
        SwerveModuleDeltas p = m_kinematics.inverse(t);
        assertEquals(Math.sqrt(2), p.frontLeft().distanceMeters(), DELTA);
        assertEquals(3 * Math.PI / 4, p.frontLeft().wrappedAngle().get().getRadians(), DELTA);
        assertEquals(Math.sqrt(2), p.frontRight().distanceMeters(), DELTA);
        assertEquals(3 * Math.PI / 4, p.frontRight().wrappedAngle().get().getRadians(), DELTA);
        assertEquals(Math.sqrt(2), p.rearLeft().distanceMeters(), DELTA);
        assertEquals(3 * Math.PI / 4, p.rearLeft().wrappedAngle().get().getRadians(), DELTA);
        assertEquals(Math.sqrt(2), p.rearRight().distanceMeters(), DELTA);
        assertEquals(3 * Math.PI / 4, p.rearRight().wrappedAngle().get().getRadians(), DELTA);
    }

    @Test
    void testRollDelta() {
        // a rotate-and-move case you can do in your head
        // face +x with right rear at origin
        // keep origin corner still, rotate around it
        // in this maneuver the steering doesn't change
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5), // 1,1
                new Translation2d(0.5, -0.5), // 1,0
                new Translation2d(-0.5, 0.5), // 0,1
                new Translation2d(-0.5, -0.5) // origin
        );
        SwerveModulePositions start = new SwerveModulePositions(
                new SwerveModulePosition100(
                        0,
                        Optional.of(Rotation2d.fromRadians(3 * Math.PI / 4))),
                new SwerveModulePosition100(
                        0,
                        Optional.of(Rotation2d.fromRadians(Math.PI / 2))),
                new SwerveModulePosition100(
                        0,
                        Optional.of(Rotation2d.fromRadians(Math.PI))),
                new SwerveModulePosition100(
                        0.0,
                        Optional.empty()));
        SwerveModulePositions end = new SwerveModulePositions(
                new SwerveModulePosition100(
                        Math.sqrt(2) * Math.PI / 2,
                        Optional.of(Rotation2d.fromRadians(3 * Math.PI / 4))),
                new SwerveModulePosition100(
                        Math.PI / 2,
                        Optional.of(Rotation2d.fromRadians(Math.PI / 2))),
                new SwerveModulePosition100(
                        Math.PI / 2,
                        Optional.of(Rotation2d.fromRadians(Math.PI))),
                new SwerveModulePosition100(
                        0,
                        Optional.empty()));

        SwerveModuleDeltas delta = SwerveModuleDeltas.modulePositionDelta(start, end);
        assertEquals(Math.sqrt(2) * Math.PI / 2, delta.frontLeft().distanceMeters(), DELTA);
        assertEquals(3 * Math.PI / 4, delta.frontLeft().wrappedAngle().get().getRadians(), DELTA);
        assertEquals(Math.PI / 2, delta.frontRight().distanceMeters(), DELTA);
        assertEquals(Math.PI / 2, delta.frontRight().wrappedAngle().get().getRadians(), DELTA);
        assertEquals(Math.PI / 2, delta.rearLeft().distanceMeters(), DELTA);
        assertEquals(Math.PI, delta.rearLeft().wrappedAngle().get().getRadians(), DELTA);
        assertEquals(0, delta.rearRight().distanceMeters(), DELTA);
        assertTrue(delta.rearRight().wrappedAngle().isEmpty());

        Twist2d twist = kinematics.forward(delta);

        assertEquals(-0.5 * Math.PI / 2, twist.dx, DELTA);
        assertEquals(0.5 * Math.PI / 2, twist.dy, DELTA);
        assertEquals(Math.PI / 2, twist.dtheta, DELTA);
    }

    @Test
    void testRoll() {
        // a rotate-and-move case you can do in your head
        // face +x with right rear at origin
        // keep origin corner still, rotate around it
        // in this maneuver the steering doesn't change
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5), // 1,1
                new Translation2d(0.5, -0.5), // 1,0
                new Translation2d(-0.5, 0.5), // 0,1
                new Translation2d(-0.5, -0.5) // origin
        );
        Twist2d twist = kinematics.forward(
                new SwerveModuleDeltas(
                        new SwerveModuleDelta(
                                Math.sqrt(2) * Math.PI / 2,
                                Optional.of(Rotation2d.fromRadians(3 * Math.PI / 4))),
                        new SwerveModuleDelta(
                                Math.PI / 2,
                                Optional.of(Rotation2d.fromRadians(Math.PI / 2))),
                        new SwerveModuleDelta(
                                Math.PI / 2,
                                Optional.of(Rotation2d.fromRadians(Math.PI))),
                        // this wheel doesn't move
                        new SwerveModuleDelta(
                                0.0,
                                Optional.empty())));

        assertEquals(-0.5 * Math.PI / 2, twist.dx, DELTA);
        assertEquals(0.5 * Math.PI / 2, twist.dy, DELTA);
        assertEquals(Math.PI / 2, twist.dtheta, DELTA);
    }

    @Test
    void testRollInverse() {
        // inverse kinematics for the above case
        SwerveDriveKinematics100 m_kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));

        // move diagonally while turning 90 degrees; this should leave
        // one of the wheels in place.
        Twist2d t = new Twist2d(
                -0.5 * Math.PI / 2,
                0.5 * Math.PI / 2,
                Math.PI / 2);

        // check that the exp is correct
        Pose2d pStart = new Pose2d(0.5, 0.5, Rotation2d.kZero);
        Pose2d pEnd = pStart.exp(t);
        assertEquals(-0.5, pEnd.getX(), DELTA);
        assertEquals(0.5, pEnd.getY(), DELTA);
        assertEquals(Math.PI / 2, pEnd.getRotation().getRadians(), DELTA);

        // check that the twist is really really correct
        Twist2d t2 = pStart.log(pEnd);
        assertEquals(t, t2);

        SwerveModuleDeltas p = m_kinematics.inverse(t);
        assertEquals(Math.sqrt(2) * Math.PI / 2, p.frontLeft().distanceMeters(), DELTA);
        assertEquals(3 * Math.PI / 4, p.frontLeft().wrappedAngle().get().getRadians(), DELTA);
        assertEquals(Math.PI / 2, p.frontRight().distanceMeters(), DELTA);
        assertEquals(Math.PI / 2, p.frontRight().wrappedAngle().get().getRadians(), DELTA);
        assertEquals(Math.PI / 2, p.rearLeft().distanceMeters(), DELTA);
        assertEquals(Math.PI, p.rearLeft().wrappedAngle().get().getRadians(), DELTA);
        // this is the one that shouldn't move
        assertEquals(0, p.rearRight().distanceMeters(), DELTA);
        assertTrue(p.rearRight().wrappedAngle().isEmpty());
    }

    /**
     * array order:
     * 
     * frontLeft
     * frontRight
     * rearLeft
     * rearRight
     */
    @Test
    void testInverse() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        assertEquals(1, kinematics.m_Jinv.get(0, 0));
        assertEquals(0, kinematics.m_Jinv.get(0, 1));
        assertEquals(-0.5, kinematics.m_Jinv.get(0, 2));
        assertEquals(0, kinematics.m_Jinv.get(1, 0));
        assertEquals(1, kinematics.m_Jinv.get(1, 1));
        assertEquals(0.5, kinematics.m_Jinv.get(1, 2));
        assertEquals(1, kinematics.m_Jinv.get(2, 0));
        assertEquals(0, kinematics.m_Jinv.get(2, 1));
        assertEquals(0.5, kinematics.m_Jinv.get(2, 2));
        assertEquals(0, kinematics.m_Jinv.get(3, 0));
        assertEquals(1, kinematics.m_Jinv.get(3, 1));
        assertEquals(0.5, kinematics.m_Jinv.get(3, 2));
    }

    /**
     * array order:
     * 
     * frontLeft
     * frontRight
     * rearLeft
     * rearRight
     */
    @Test
    void testForward() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        assertEquals(0.25, kinematics.m_J.get(0, 0), DELTA);
        assertEquals(0, kinematics.m_J.get(1, 0), DELTA);
        assertEquals(-0.25, kinematics.m_J.get(2, 0), DELTA);
        assertEquals(0, kinematics.m_J.get(0, 1), DELTA);
        assertEquals(0.25, kinematics.m_J.get(1, 1), DELTA);
        assertEquals(0.25, kinematics.m_J.get(2, 1), DELTA);
        assertEquals(0.25, kinematics.m_J.get(0, 2), DELTA);
        assertEquals(0, kinematics.m_J.get(1, 2), DELTA);
        assertEquals(0.25, kinematics.m_J.get(2, 2), DELTA);
        assertEquals(0, kinematics.m_J.get(0, 3), DELTA);
        assertEquals(0.25, kinematics.m_J.get(1, 3), DELTA);
        assertEquals(0.25, kinematics.m_J.get(2, 3), DELTA);
    }

    @Test
    void testTwistStraight() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        // 0.1m straight ahead, all same.
        Twist2d twist = kinematics.forward(
                new SwerveModuleDeltas(
                        new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                        new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                        new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                        new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0)))));

        assertEquals(0.1, twist.dx, DELTA);
        assertEquals(0, twist.dy, DELTA);
        assertEquals(0, twist.dtheta, DELTA);
    }

    @Test
    void testTwistSpin() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));

        Twist2d twist = kinematics.forward(
                new SwerveModuleDeltas(
                        new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(135))),
                        new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(45))),
                        new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(-135))),
                        new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(-45)))));

        assertEquals(0, twist.dx, DELTA);
        assertEquals(0, twist.dy, DELTA);
        assertEquals(0.141, twist.dtheta, DELTA);
    }

    /**
     * The WPI kinematics class keeps module state so that it can return
     * old steering values for future zero-speed cases, but this seems like
     * a bad place to put the state. Instead, this memory should be handled
     * closer to the actuators, because that's all we're trying to optimize.
     */
    @Test
    void testStopped() {
        SwerveDriveKinematics100 k = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        ChassisSpeeds s = new ChassisSpeeds(0, 1, 0);
        // this sets the steering
        SwerveModuleStates m = k.inverse(SwerveKinodynamics.discretize(s, 0.02));
        assertEquals(1.571, m.frontLeft().angle().get().getRadians(), DELTA);
        assertEquals(1, m.frontLeft().speed(), DELTA);
        s = new ChassisSpeeds(0, 0, 0);
        // this used to be the same even though the velocity is zero.
        // now it's just empty.
        m = k.inverse(SwerveKinodynamics.discretize(s, 0.02));
        assertTrue(m.frontLeft().angle().isEmpty());
        assertEquals(0, m.frontLeft().speed(), DELTA);
    }

    @Test
    void testStoppedDelta() {
        SwerveDriveKinematics100 k = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        Twist2d s = new Twist2d(0, 1, 0);
        // this sets the steering
        SwerveModuleDeltas m = k.inverse(s);
        assertEquals(1.571, m.frontLeft().wrappedAngle().get().getRadians(), DELTA);
        assertEquals(1, m.frontLeft().distanceMeters(), DELTA);
        s = new Twist2d(0, 0, 0);
        // there's no positional state anymore so steering is empty.
        m = k.inverse(s);
        assertTrue(m.frontLeft().wrappedAngle().isEmpty());
        assertEquals(0, m.frontLeft().distanceMeters(), DELTA);
    }

    @Test
    void testWithTime() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        // 0.1m straight ahead, all same.
        Twist2d twist = kinematics.forward(
                new SwerveModuleDeltas(
                        new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                        new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                        new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                        new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0)))));

        assertEquals(0.1, twist.dx, DELTA);
        assertEquals(0, twist.dy, DELTA);
        assertEquals(0, twist.dtheta, DELTA);
    }

    ////////////////////////////////////////
    //
    // tests below are from WPILib
    //
    //

    private static final double EPSILON = 1E-9;

    private final Translation2d m_fl = new Translation2d(12, 12);
    private final Translation2d m_fr = new Translation2d(12, -12);
    private final Translation2d m_bl = new Translation2d(-12, 12);
    private final Translation2d m_br = new Translation2d(-12, -12);

    private final SwerveDriveKinematics100 m_kinematics = new SwerveDriveKinematics100(m_fl, m_fr, m_bl, m_br);

    @Test
    void testStraightLineInverseKinematics() { // test inverse kinematics going in a straight line

        ChassisSpeeds speeds = new ChassisSpeeds(5, 0, 0);
        var moduleStates = m_kinematics.inverse(SwerveKinodynamics.discretize(speeds, 0.02));

        assertAll(
                () -> assertEquals(5.0, moduleStates.frontLeft().speed(), EPSILON),
                () -> assertEquals(5.0, moduleStates.frontRight().speed(), EPSILON),
                () -> assertEquals(5.0, moduleStates.rearLeft().speed(), EPSILON),
                () -> assertEquals(5.0, moduleStates.rearRight().speed(), EPSILON),
                () -> assertEquals(0.0, moduleStates.frontLeft().angle().get().getRadians(), EPSILON),
                () -> assertEquals(0.0, moduleStates.frontRight().angle().get().getRadians(), EPSILON),
                () -> assertEquals(0.0, moduleStates.rearLeft().angle().get().getRadians(), EPSILON),
                () -> assertEquals(0.0, moduleStates.rearRight().angle().get().getRadians(), EPSILON));
    }

    @Test
    void testStraightLineForwardKinematics() { // test forward kinematics going in a straight line
        SwerveModuleState100 state = new SwerveModuleState100(5.0, Optional.of(Rotation2d.fromDegrees(0.0)));
        DiscreteSpeed chassisSpeeds = m_kinematics.forward(
                new SwerveModuleStates(state, state, state, state), 0.02);

        assertAll(
                () -> assertEquals(5.0, chassisSpeeds.twist().dx / 0.02, EPSILON),
                () -> assertEquals(0.0, chassisSpeeds.twist().dy / 0.02, EPSILON),
                () -> assertEquals(0.0, chassisSpeeds.twist().dtheta / 0.02, EPSILON));
    }

    @Test
    void testStraightLineForwardKinematicsWithDeltas() {
        // test forward kinematics going in a straight line
        SwerveModuleDelta delta = new SwerveModuleDelta(5.0, Optional.of(Rotation2d.fromDegrees(0.0)));
        Twist2d twist = m_kinematics.forward(
                new SwerveModuleDeltas(delta, delta, delta, delta));

        assertEquals(5.0, twist.dx, EPSILON);
        assertEquals(0.0, twist.dy, EPSILON);
        assertEquals(0.0, twist.dtheta, EPSILON);
    }

    @Test
    void testStraightStrafeInverseKinematics() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 5, 0);
        SwerveModuleStates moduleStates = m_kinematics
                .inverse(SwerveKinodynamics.discretize(speeds, 0.02));

        assertAll(
                () -> assertEquals(5.0, moduleStates.frontLeft().speed(), EPSILON),
                () -> assertEquals(5.0, moduleStates.frontRight().speed(), EPSILON),
                () -> assertEquals(5.0, moduleStates.rearLeft().speed(), EPSILON),
                () -> assertEquals(5.0, moduleStates.rearRight().speed(), EPSILON),
                () -> assertEquals(90.0, moduleStates.frontLeft().angle().get().getDegrees(), EPSILON),
                () -> assertEquals(90.0, moduleStates.frontRight().angle().get().getDegrees(), EPSILON),
                () -> assertEquals(90.0, moduleStates.rearLeft().angle().get().getDegrees(), EPSILON),
                () -> assertEquals(90.0, moduleStates.rearRight().angle().get().getDegrees(), EPSILON));
    }

    @Test
    void testStraightStrafeForwardKinematics() {
        SwerveModuleState100 state = new SwerveModuleState100(5.0, Optional.of(Rotation2d.fromDegrees(90.0)));
        DiscreteSpeed discreteSpeed = m_kinematics.forward(
                new SwerveModuleStates(state, state, state, state), 0.02);

        assertAll(
                () -> assertEquals(0.0, discreteSpeed.twist().dx / 0.02, EPSILON),
                () -> assertEquals(5.0, discreteSpeed.twist().dy / 0.02, EPSILON),
                () -> assertEquals(0.0, discreteSpeed.twist().dtheta / 0.02, EPSILON));
    }

    @Test
    void testStraightStrafeForwardKinematicsWithDeltas() {
        SwerveModuleDelta delta = new SwerveModuleDelta(
                5.0,
                Optional.of(Rotation2d.fromDegrees(90.0)));
        Twist2d twist = m_kinematics.forward(
                new SwerveModuleDeltas(delta, delta, delta, delta));

        assertEquals(0.0, twist.dx, EPSILON);
        assertEquals(5.0, twist.dy, EPSILON);
        assertEquals(0.0, twist.dtheta, EPSILON);
    }

    @Test
    void testConserveWheelAngle() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 2 * Math.PI);
        m_kinematics.inverse(SwerveKinodynamics.discretize(speeds, 0.02));
        var moduleStates = m_kinematics.inverse(SwerveKinodynamics.discretize(
                new ChassisSpeeds(), 0.02));

        // This used to preserve module angles.
        // Now it returns empty angles, and the right thing happens downstream.

        assertAll(
                () -> assertEquals(0.0, moduleStates.frontLeft().speed(), EPSILON),
                () -> assertEquals(0.0, moduleStates.frontRight().speed(), EPSILON),
                () -> assertEquals(0.0, moduleStates.rearLeft().speed(), EPSILON),
                () -> assertEquals(0.0, moduleStates.rearRight().speed(), EPSILON),
                () -> assertTrue(moduleStates.frontLeft().angle().isEmpty()),
                () -> assertTrue(moduleStates.frontRight().angle().isEmpty()),
                () -> assertTrue(moduleStates.rearLeft().angle().isEmpty()),
                () -> assertTrue(moduleStates.rearRight().angle().isEmpty()));
    }

    @Test
    void testResetWheelAngle() {
        SwerveModuleStates moduleStates = m_kinematics.inverse(
                SwerveKinodynamics.discretize(new ChassisSpeeds(), 0.02));
        // Robot is stationary, so module angles are empty.
        assertAll(
                () -> assertEquals(0.0, moduleStates.frontLeft().speed(), EPSILON),
                () -> assertEquals(0.0, moduleStates.frontRight().speed(), EPSILON),
                () -> assertEquals(0.0, moduleStates.rearLeft().speed(), EPSILON),
                () -> assertEquals(0.0, moduleStates.rearRight().speed(), EPSILON),
                () -> assertTrue(moduleStates.frontLeft().angle().isEmpty()),
                () -> assertTrue(moduleStates.frontRight().angle().isEmpty()),
                () -> assertTrue(moduleStates.rearLeft().angle().isEmpty()),
                () -> assertTrue(moduleStates.rearRight().angle().isEmpty()));
    }

    @Test
    void testTurnInPlaceInverseKinematics() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 2 * Math.PI);
        SwerveModuleStates moduleStates = m_kinematics.inverse(
                SwerveKinodynamics.discretize(speeds, 0.02));

        /*
         * The circumference of the wheels about the COR is pi * diameter, or 2pi *
         * radius
         * the radius is the sqrt(12*12in + 12*12in), or 16.9706in, so the circumference
         * the
         * wheels
         * trace out is 106.629190516in. since we want our robot to rotate at 1 rotation
         * per second,
         * our wheels must trace out 1 rotation (or 106.63 inches) per second.
         */

        assertAll(
                () -> assertEquals(106.63, moduleStates.frontLeft().speed(), 0.1),
                () -> assertEquals(106.63, moduleStates.frontRight().speed(), 0.1),
                () -> assertEquals(106.63, moduleStates.rearLeft().speed(), 0.1),
                () -> assertEquals(106.63, moduleStates.rearRight().speed(), 0.1),
                () -> assertEquals(135.0, moduleStates.frontLeft().angle().get().getDegrees(), EPSILON),
                () -> assertEquals(45.0, moduleStates.frontRight().angle().get().getDegrees(), EPSILON),
                () -> assertEquals(-135.0, moduleStates.rearLeft().angle().get().getDegrees(), EPSILON),
                () -> assertEquals(-45.0, moduleStates.rearRight().angle().get().getDegrees(), EPSILON));
    }

    @Test
    void testTurnInPlaceForwardKinematics() {
        SwerveModuleState100 flState = new SwerveModuleState100(106.629, Optional.of(Rotation2d.fromDegrees(135)));
        SwerveModuleState100 frState = new SwerveModuleState100(106.629, Optional.of(Rotation2d.fromDegrees(45)));
        SwerveModuleState100 blState = new SwerveModuleState100(106.629, Optional.of(Rotation2d.fromDegrees(-135)));
        SwerveModuleState100 brState = new SwerveModuleState100(106.629, Optional.of(Rotation2d.fromDegrees(-45)));
        DiscreteSpeed chassisSpeeds = m_kinematics.forward(
                new SwerveModuleStates(flState, frState, blState, brState), 0.02);

        assertAll(
                () -> assertEquals(0.0, chassisSpeeds.twist().dx / 0.02, EPSILON),
                () -> assertEquals(0.0, chassisSpeeds.twist().dy / 0.02, EPSILON),
                () -> assertEquals(2 * Math.PI, chassisSpeeds.twist().dtheta / 0.02, 0.1));
    }

    @Test
    void testTurnInPlaceForwardKinematicsWithDeltas() {
        SwerveModuleDelta flDelta = new SwerveModuleDelta(106.629,
                Optional.of(Rotation2d.fromDegrees(135)));
        SwerveModuleDelta frDelta = new SwerveModuleDelta(106.629,
                Optional.of(Rotation2d.fromDegrees(45)));
        SwerveModuleDelta blDelta = new SwerveModuleDelta(106.629,
                Optional.of(Rotation2d.fromDegrees(-135)));
        SwerveModuleDelta brDelta = new SwerveModuleDelta(106.629,
                Optional.of(Rotation2d.fromDegrees(-45)));

        Twist2d twist = m_kinematics.forward(
                new SwerveModuleDeltas(
                        flDelta, frDelta, blDelta, brDelta));

        assertEquals(0.0, twist.dx, EPSILON);
        assertEquals(0.0, twist.dy, EPSILON);
        assertEquals(2 * Math.PI, twist.dtheta, 0.1);
    }

    @Test
    void testOffCenterCORRotationForwardKinematics() {
        SwerveModuleState100 flState = new SwerveModuleState100(0.0, Optional.of(Rotation2d.fromDegrees(0.0)));
        SwerveModuleState100 frState = new SwerveModuleState100(150.796, Optional.of(Rotation2d.fromDegrees(0.0)));
        SwerveModuleState100 blState = new SwerveModuleState100(150.796, Optional.of(Rotation2d.fromDegrees(-90)));
        SwerveModuleState100 brState = new SwerveModuleState100(213.258, Optional.of(Rotation2d.fromDegrees(-45)));
        DiscreteSpeed chassisSpeeds = m_kinematics.forward(
                new SwerveModuleStates(flState, frState, blState, brState), 0.02);

        /*
         * We already know that our omega should be 2π from the previous test. Next, we
         * need to determine
         * the vx and vy of our chassis center. Because our COR is at a 45 degree angle
         * from the center,
         * we know that vx and vy must be the same. Furthermore, we know that the center
         * of mass makes
         * a full revolution about the center of revolution once every second.
         * Therefore, the center of
         * mass must be moving at 106.629in/sec. Recalling that the ratios of a 45/45/90
         * triangle are
         * 1:√(2)/2:√(2)/2, we find that the COM vx is -75.398, and vy is 75.398.
         */

        assertAll(
                () -> assertEquals(75.398, chassisSpeeds.twist().dx / 0.02, 0.1),
                () -> assertEquals(-75.398, chassisSpeeds.twist().dy / 0.02, 0.1),
                () -> assertEquals(2 * Math.PI, chassisSpeeds.twist().dtheta / 0.02, 0.1));
    }

    @Test
    void testOffCenterCORRotationForwardKinematicsWithDeltas() {
        SwerveModuleDelta flDelta = new SwerveModuleDelta(0.0,
                Optional.of(Rotation2d.fromDegrees(0.0)));
        SwerveModuleDelta frDelta = new SwerveModuleDelta(150.796,
                Optional.of(Rotation2d.fromDegrees(0.0)));
        SwerveModuleDelta blDelta = new SwerveModuleDelta(150.796,
                Optional.of(Rotation2d.fromDegrees(-90)));
        SwerveModuleDelta brDelta = new SwerveModuleDelta(213.258,
                Optional.of(Rotation2d.fromDegrees(-45)));

        Twist2d twist = m_kinematics.forward(
                new SwerveModuleDeltas(flDelta, frDelta, blDelta, brDelta));

        /*
         * We already know that our omega should be 2π from the previous test. Next, we
         * need to determine
         * the vx and vy of our chassis center. Because our COR is at a 45 degree angle
         * from the center,
         * we know that vx and vy must be the same. Furthermore, we know that the center
         * of mass makes
         * a full revolution about the center of revolution once every second.
         * Therefore, the center of
         * mass must be moving at 106.629in/sec. Recalling that the ratios of a 45/45/90
         * triangle are
         * 1:√(2)/2:√(2)/2, we find that the COM vx is -75.398, and vy is 75.398.
         */

        assertAll(
                () -> assertEquals(75.398, twist.dx, 0.1),
                () -> assertEquals(-75.398, twist.dy, 0.1),
                () -> assertEquals(2 * Math.PI, twist.dtheta, 0.1));
    }

    @Test
    void testOffCenterCORRotationAndTranslationForwardKinematics() {
        SwerveModuleState100 flState = new SwerveModuleState100(23.43, Optional.of(Rotation2d.fromDegrees(-140.19)));
        SwerveModuleState100 frState = new SwerveModuleState100(23.43, Optional.of(Rotation2d.fromDegrees(-39.81)));
        SwerveModuleState100 blState = new SwerveModuleState100(54.08, Optional.of(Rotation2d.fromDegrees(-109.44)));
        SwerveModuleState100 brState = new SwerveModuleState100(54.08, Optional.of(Rotation2d.fromDegrees(-70.56)));
        DiscreteSpeed chassisSpeeds = m_kinematics.forward(
                new SwerveModuleStates(flState, frState, blState, brState), 0.02);

        /*
         * From equation (13.17), we know that chassis motion is th dot product of the
         * pseudoinverse of the inverseKinematics matrix (with the center of rotation at
         * (0,0) -- we don't want the motion of the center of rotation, we want it of
         * the center of the robot). These above SwerveModuleState100s are known to be
         * from
         * a velocity of [[0][3][1.5]] about (0, 24), and the expected numbers have been
         * calculated using Numpy's linalg.pinv function.
         */

        assertEquals(0.0, chassisSpeeds.twist().dx / 0.02, 0.1);
        assertEquals(-33.0, chassisSpeeds.twist().dy / 0.02, 0.1);
        assertEquals(1.5, chassisSpeeds.twist().dtheta / 0.02, 0.1);
    }

    @Test
    void testOffCenterCORRotationAndTranslationForwardKinematicsWithDeltas() {
        SwerveModuleDelta flDelta = new SwerveModuleDelta(23.43,
                Optional.of(Rotation2d.fromDegrees(-140.19)));
        SwerveModuleDelta frDelta = new SwerveModuleDelta(23.43,
                Optional.of(Rotation2d.fromDegrees(-39.81)));
        SwerveModuleDelta blDelta = new SwerveModuleDelta(54.08,
                Optional.of(Rotation2d.fromDegrees(-109.44)));
        SwerveModuleDelta brDelta = new SwerveModuleDelta(54.08,
                Optional.of(Rotation2d.fromDegrees(-70.56)));

        Twist2d twist = m_kinematics.forward(
                new SwerveModuleDeltas(flDelta, frDelta, blDelta, brDelta));

        /*
         * From equation (13.17), we know that chassis motion is th dot product of the
         * pseudoinverse of the inverseKinematics matrix (with the center of rotation at
         * (0,0) -- we don't want the motion of the center of rotation, we want it of
         * the center of the robot). These above SwerveModuleState100s are known to be
         * from
         * a velocity of [[0][3][1.5]] about (0, 24), and the expected numbers have been
         * calculated using Numpy's linalg.pinv function.
         */

        assertAll(
                () -> assertEquals(0.0, twist.dx, 0.1),
                () -> assertEquals(-33.0, twist.dy, 0.1),
                () -> assertEquals(1.5, twist.dtheta, 0.1));
    }

}
