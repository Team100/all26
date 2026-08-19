package org.team100.lib.kinematics.mecanum;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.kinematics.mecanum.MecanumKinematics100.Slip;
import org.team100.lib.testing.Timeless;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.MecanumDriveKinematics;
import org.wpilib.math.kinematics.MecanumDriveWheelPositions;
import org.wpilib.math.kinematics.MecanumDriveWheelVelocities;

public class MecanumKinematics100Test implements Timeless{
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    @Test
    void testWPI() {
        // no corrections in the WPI version.
        MecanumDriveKinematics k = new MecanumDriveKinematics(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        // all ahead
        Twist2d t = k.toTwist2d(
                new MecanumDriveWheelPositions(),
                new MecanumDriveWheelPositions(0.1, 0.1, 0.1, 0.1));
        assertEquals(0.1, t.dx, DELTA);
        assertEquals(0.0, t.dy, DELTA);
        assertEquals(0.0, t.dtheta, DELTA);

        // strafe left
        t = k.toTwist2d(
                new MecanumDriveWheelPositions(),
                new MecanumDriveWheelPositions(-0.1, 0.1, 0.1, -0.1));
        assertEquals(0.0, t.dx, DELTA);
        assertEquals(0.1, t.dy, DELTA);
        assertEquals(0.0, t.dtheta, DELTA);

        // spin CCW
        t = k.toTwist2d(
                new MecanumDriveWheelPositions(),
                new MecanumDriveWheelPositions(-0.1, 0.1, -0.1, 0.1));
        assertEquals(0.0, t.dx, DELTA);
        assertEquals(0.0, t.dy, DELTA);
        assertEquals(0.1, t.dtheta, DELTA);

        // all ahead
        MecanumDriveWheelVelocities s = k.toWheelVelocities(new ChassisVelocities(1, 0, 0));
        assertEquals(1, s.frontLeft, DELTA);
        assertEquals(1, s.frontRight, DELTA);
        assertEquals(1, s.rearLeft, DELTA);
        assertEquals(1, s.rearRight, DELTA);

        // strafe left
        s = k.toWheelVelocities(new ChassisVelocities(0, 1, 0));
        assertEquals(-1, s.frontLeft, DELTA);
        assertEquals(1, s.frontRight, DELTA);
        assertEquals(1, s.rearLeft, DELTA);
        assertEquals(-1, s.rearRight, DELTA);

        // diagonal?
        s = k.toWheelVelocities(new ChassisVelocities(1, 1, 0));
        assertEquals(0, s.frontLeft, DELTA);
        assertEquals(2, s.frontRight, DELTA);
        assertEquals(2, s.rearLeft, DELTA);
        assertEquals(0, s.rearRight, DELTA);

        // spin CCW
        s = k.toWheelVelocities(new ChassisVelocities(0, 0, 1));
        assertEquals(-1, s.frontLeft, DELTA);
        assertEquals(1, s.frontRight, DELTA);
        assertEquals(-1, s.rearLeft, DELTA);
        assertEquals(1, s.rearRight, DELTA);
    }

    @Test
    void testWPIEnvelope() {
        MecanumDriveKinematics k = new MecanumDriveKinematics(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        if (DEBUG)
            System.out.println("theta, speed");
        for (double theta = 0; theta < 2 * Math.PI; theta += 0.1) {
            Rotation2d r = new Rotation2d(theta);
            MecanumDriveWheelVelocities s = k.toWheelVelocities(
                    new ChassisVelocities(r.getCos(), r.getSin(), 0));
            double maxWheelSpeed = Math.max(
                    Math.max(
                            Math.abs(s.frontLeft),
                            Math.abs(s.frontRight)),
                    Math.max(
                            Math.abs(s.rearLeft),
                            Math.abs(s.rearRight)));
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f\n", theta, 1 / maxWheelSpeed);
        }
    }

    @Test
    void testUncorrected() {
        // all correction factors 1 => same as above.
        MecanumKinematics100 k = new MecanumKinematics100(
                new Slip(1, 1, 1),
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        // all ahead
        Twist2d t = k.toTwist2d(
                new MecanumDriveWheelPositions(),
                new MecanumDriveWheelPositions(0.1, 0.1, 0.1, 0.1));
        assertEquals(0.1, t.dx, DELTA);
        assertEquals(0.0, t.dy, DELTA);
        assertEquals(0.0, t.dtheta, DELTA);

        // strafe left
        t = k.toTwist2d(
                new MecanumDriveWheelPositions(),
                new MecanumDriveWheelPositions(-0.1, 0.1, 0.1, -0.1));
        assertEquals(0.0, t.dx, DELTA);
        assertEquals(0.1, t.dy, DELTA);
        assertEquals(0.0, t.dtheta, DELTA);

        // spin CCW
        t = k.toTwist2d(
                new MecanumDriveWheelPositions(),
                new MecanumDriveWheelPositions(-0.1, 0.1, -0.1, 0.1));
        assertEquals(0.0, t.dx, DELTA);
        assertEquals(0.0, t.dy, DELTA);
        assertEquals(0.1, t.dtheta, DELTA);

        // all ahead
        MecanumDriveWheelVelocities s = k.toWheelVelocities(new ChassisVelocities(1, 0, 0));
        assertEquals(1, s.frontLeft, DELTA);
        assertEquals(1, s.frontRight, DELTA);
        assertEquals(1, s.rearLeft, DELTA);
        assertEquals(1, s.rearRight, DELTA);

        // strafe left
        s = k.toWheelVelocities(new ChassisVelocities(0, 1, 0));
        assertEquals(-1, s.frontLeft, DELTA);
        assertEquals(1, s.frontRight, DELTA);
        assertEquals(1, s.rearLeft, DELTA);
        assertEquals(-1, s.rearRight, DELTA);

        // spin CCW
        s = k.toWheelVelocities(new ChassisVelocities(0, 0, 1));
        assertEquals(-1, s.frontLeft, DELTA);
        assertEquals(1, s.frontRight, DELTA);
        assertEquals(-1, s.rearLeft, DELTA);
        assertEquals(1, s.rearRight, DELTA);
    }

    @Test
    void testCorrected() {
        // most likely case: strafing corrected, others 1.
        MecanumKinematics100 k = new MecanumKinematics100(
                new Slip(1, 1.5, 1),
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        // all ahead, no change
        Twist2d t = k.toTwist2d(
                new MecanumDriveWheelPositions(),
                new MecanumDriveWheelPositions(0.1, 0.1, 0.1, 0.1));
        assertEquals(0.1, t.dx, DELTA);
        assertEquals(0.0, t.dy, DELTA);
        assertEquals(0.0, t.dtheta, DELTA);

        // strafe left: true (slipping) wheel positions here, get scaled down to true
        // speed
        t = k.toTwist2d(
                new MecanumDriveWheelPositions(),
                new MecanumDriveWheelPositions(-0.15, 0.15, 0.15, -0.15));
        assertEquals(0.0, t.dx, DELTA);
        assertEquals(0.1, t.dy, DELTA);
        assertEquals(0.0, t.dtheta, DELTA);

        // spin CCW, no change
        t = k.toTwist2d(
                new MecanumDriveWheelPositions(),
                new MecanumDriveWheelPositions(-0.1, 0.1, -0.1, 0.1));
        assertEquals(0.0, t.dx, DELTA);
        assertEquals(0.0, t.dy, DELTA);
        assertEquals(0.1, t.dtheta, DELTA);

        // all ahead, no change
        MecanumDriveWheelVelocities s = k.toWheelVelocities(new ChassisVelocities(1, 0, 0));
        assertEquals(1, s.frontLeft, DELTA);
        assertEquals(1, s.frontRight, DELTA);
        assertEquals(1, s.rearLeft, DELTA);
        assertEquals(1, s.rearRight, DELTA);

        // strafe left: wheels go faster
        s = k.toWheelVelocities(new ChassisVelocities(0, 1, 0));
        assertEquals(-1.5, s.frontLeft, DELTA);
        assertEquals(1.5, s.frontRight, DELTA);
        assertEquals(1.5, s.rearLeft, DELTA);
        assertEquals(-1.5, s.rearRight, DELTA);

        // diagonal?
        s = k.toWheelVelocities(new ChassisVelocities(1, 1, 0));
        assertEquals(-0.5, s.frontLeft, DELTA);
        assertEquals(2.5, s.frontRight, DELTA);
        assertEquals(2.5, s.rearLeft, DELTA);
        assertEquals(-0.5, s.rearRight, DELTA);

        // spin CCW, no change
        s = k.toWheelVelocities(new ChassisVelocities(0, 0, 1));
        assertEquals(-1, s.frontLeft, DELTA);
        assertEquals(1, s.frontRight, DELTA);
        assertEquals(-1, s.rearLeft, DELTA);
        assertEquals(1, s.rearRight, DELTA);
    }

    @Test
    void testCorrectedEnvelope() {
        MecanumKinematics100 k = new MecanumKinematics100(
                new Slip(1, 1.5, 1),
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        if (DEBUG)
            System.out.println("theta, speed");
        for (double theta = 0; theta < 2 * Math.PI; theta += Math.PI / 30) {
            Rotation2d r = new Rotation2d(theta);
            MecanumDriveWheelVelocities s = k.toWheelVelocities(
                    new ChassisVelocities(r.getCos(), r.getSin(), 0));
            double maxWheelSpeed = Math.max(
                    Math.max(
                            Math.abs(s.frontLeft),
                            Math.abs(s.frontRight)),
                    Math.max(
                            Math.abs(s.rearLeft),
                            Math.abs(s.rearRight)));
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f\n", theta, 1 / maxWheelSpeed);
        }
    }

}
