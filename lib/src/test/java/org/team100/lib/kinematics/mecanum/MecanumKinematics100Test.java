package org.team100.lib.kinematics.mecanum;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.kinematics.mecanum.MecanumKinematics100.Slip;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

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
        MecanumDriveWheelSpeeds s = k.toWheelSpeeds(new ChassisSpeeds(1, 0, 0));
        assertEquals(1, s.frontLeftMetersPerSecond, DELTA);
        assertEquals(1, s.frontRightMetersPerSecond, DELTA);
        assertEquals(1, s.rearLeftMetersPerSecond, DELTA);
        assertEquals(1, s.rearRightMetersPerSecond, DELTA);

        // strafe left
        s = k.toWheelSpeeds(new ChassisSpeeds(0, 1, 0));
        assertEquals(-1, s.frontLeftMetersPerSecond, DELTA);
        assertEquals(1, s.frontRightMetersPerSecond, DELTA);
        assertEquals(1, s.rearLeftMetersPerSecond, DELTA);
        assertEquals(-1, s.rearRightMetersPerSecond, DELTA);

        // diagonal?
        s = k.toWheelSpeeds(new ChassisSpeeds(1, 1, 0));
        assertEquals(0, s.frontLeftMetersPerSecond, DELTA);
        assertEquals(2, s.frontRightMetersPerSecond, DELTA);
        assertEquals(2, s.rearLeftMetersPerSecond, DELTA);
        assertEquals(0, s.rearRightMetersPerSecond, DELTA);

        // spin CCW
        s = k.toWheelSpeeds(new ChassisSpeeds(0, 0, 1));
        assertEquals(-1, s.frontLeftMetersPerSecond, DELTA);
        assertEquals(1, s.frontRightMetersPerSecond, DELTA);
        assertEquals(-1, s.rearLeftMetersPerSecond, DELTA);
        assertEquals(1, s.rearRightMetersPerSecond, DELTA);
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
            MecanumDriveWheelSpeeds s = k.toWheelSpeeds(
                    new ChassisSpeeds(r.getCos(), r.getSin(), 0));
            double maxWheelSpeed = Math.max(
                    Math.max(
                            Math.abs(s.frontLeftMetersPerSecond),
                            Math.abs(s.frontRightMetersPerSecond)),
                    Math.max(
                            Math.abs(s.rearLeftMetersPerSecond),
                            Math.abs(s.rearRightMetersPerSecond)));
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
        MecanumDriveWheelSpeeds s = k.toWheelSpeeds(new ChassisSpeeds(1, 0, 0));
        assertEquals(1, s.frontLeftMetersPerSecond, DELTA);
        assertEquals(1, s.frontRightMetersPerSecond, DELTA);
        assertEquals(1, s.rearLeftMetersPerSecond, DELTA);
        assertEquals(1, s.rearRightMetersPerSecond, DELTA);

        // strafe left
        s = k.toWheelSpeeds(new ChassisSpeeds(0, 1, 0));
        assertEquals(-1, s.frontLeftMetersPerSecond, DELTA);
        assertEquals(1, s.frontRightMetersPerSecond, DELTA);
        assertEquals(1, s.rearLeftMetersPerSecond, DELTA);
        assertEquals(-1, s.rearRightMetersPerSecond, DELTA);

        // spin CCW
        s = k.toWheelSpeeds(new ChassisSpeeds(0, 0, 1));
        assertEquals(-1, s.frontLeftMetersPerSecond, DELTA);
        assertEquals(1, s.frontRightMetersPerSecond, DELTA);
        assertEquals(-1, s.rearLeftMetersPerSecond, DELTA);
        assertEquals(1, s.rearRightMetersPerSecond, DELTA);
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
        MecanumDriveWheelSpeeds s = k.toWheelSpeeds(new ChassisSpeeds(1, 0, 0));
        assertEquals(1, s.frontLeftMetersPerSecond, DELTA);
        assertEquals(1, s.frontRightMetersPerSecond, DELTA);
        assertEquals(1, s.rearLeftMetersPerSecond, DELTA);
        assertEquals(1, s.rearRightMetersPerSecond, DELTA);

        // strafe left: wheels go faster
        s = k.toWheelSpeeds(new ChassisSpeeds(0, 1, 0));
        assertEquals(-1.5, s.frontLeftMetersPerSecond, DELTA);
        assertEquals(1.5, s.frontRightMetersPerSecond, DELTA);
        assertEquals(1.5, s.rearLeftMetersPerSecond, DELTA);
        assertEquals(-1.5, s.rearRightMetersPerSecond, DELTA);

        // diagonal?
        s = k.toWheelSpeeds(new ChassisSpeeds(1, 1, 0));
        assertEquals(-0.5, s.frontLeftMetersPerSecond, DELTA);
        assertEquals(2.5, s.frontRightMetersPerSecond, DELTA);
        assertEquals(2.5, s.rearLeftMetersPerSecond, DELTA);
        assertEquals(-0.5, s.rearRightMetersPerSecond, DELTA);

        // spin CCW, no change
        s = k.toWheelSpeeds(new ChassisSpeeds(0, 0, 1));
        assertEquals(-1, s.frontLeftMetersPerSecond, DELTA);
        assertEquals(1, s.frontRightMetersPerSecond, DELTA);
        assertEquals(-1, s.rearLeftMetersPerSecond, DELTA);
        assertEquals(1, s.rearRightMetersPerSecond, DELTA);
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
            MecanumDriveWheelSpeeds s = k.toWheelSpeeds(
                    new ChassisSpeeds(r.getCos(), r.getSin(), 0));
            double maxWheelSpeed = Math.max(
                    Math.max(
                            Math.abs(s.frontLeftMetersPerSecond),
                            Math.abs(s.frontRightMetersPerSecond)),
                    Math.max(
                            Math.abs(s.rearLeftMetersPerSecond),
                            Math.abs(s.rearRightMetersPerSecond)));
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f\n", theta, 1 / maxWheelSpeed);
        }
    }

}
