package org.team100.lib.camera;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.localization.Blip;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

/**
 * Remind myself how 3d rotations work.
 */
class CameraTest {
    private static final double DELTA = 0.01;

    @Test
    void testCalibration0() {
        // view in camera: straight ahead (z), one meter away, no rotation
        Blip b = new Blip(0, 0, new Transform3d(0, 0, 1, new Rotation3d()));
        // x-forward version of that
        Transform3d cameraToTag = b.blipToTransform();
        Transform3d robotToTag = new Transform3d(1, 0, 0, new Rotation3d());
        Transform3d tagToCamera = cameraToTag.inverse();
        Transform3d robotToCamera = robotToTag.plus(tagToCamera);
        assertEquals(0, robotToCamera.getX(), DELTA);
        assertEquals(0, robotToCamera.getY(), DELTA);
        assertEquals(0, robotToCamera.getZ(), DELTA);
        Rotation3d or = robotToCamera.getRotation();
        assertEquals(0, or.getX(), DELTA);
        assertEquals(0, or.getY(), DELTA);
        assertEquals(0, or.getZ(), DELTA);
    }

    @Test
    void testNoRotation() {
        double roll = 0.0; // around X (ahead)
        double pitch = 0.0; // around Y (left)
        double yaw = 0.0; // around Z (up)
        Rotation3d r = new Rotation3d(roll, pitch, yaw);
        assertEquals(0, r.getX(), DELTA);
        assertEquals(0, r.getY(), DELTA);
        assertEquals(0, r.getZ(), DELTA);
    }

    @Test
    void testRoll() {
        double roll = 1.0; // clockwise
        double pitch = 0.0;
        double yaw = 0.0;
        Rotation3d r = new Rotation3d(roll, pitch, yaw);
        assertEquals(1.0, r.getX(), DELTA);
        assertEquals(0, r.getY(), DELTA);
        assertEquals(0, r.getZ(), DELTA);
    }

    @Test
    void testPitch() {
        double roll = 0.0;
        double pitch = 1.0; // down
        double yaw = 0.0;
        Rotation3d r = new Rotation3d(roll, pitch, yaw);
        assertEquals(0, r.getX(), DELTA);
        assertEquals(1.0, r.getY(), DELTA);
        assertEquals(0, r.getZ(), DELTA);
    }

    @Test
    void testYaw() {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 1.0; // left
        Rotation3d r = new Rotation3d(roll, pitch, yaw);
        assertEquals(0, r.getX(), DELTA);
        assertEquals(0, r.getY(), DELTA);
        assertEquals(1.0, r.getZ(), DELTA);
    }

    @Test
    void testIMatrix() {
        Matrix<N3, N3> matrix = Matrix.eye(Nat.N3());
        Rotation3d r = new Rotation3d(matrix);
        assertEquals(0, r.getX(), DELTA);
        assertEquals(0, r.getY(), DELTA);
        assertEquals(0, r.getZ(), DELTA);
    }

    @Test
    void testRollMatrix() {
        // in FRC X is the roll axis, positive roll is clockwise looking in the X
        // direction
        Matrix<N3, N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        double rot = Math.sqrt(2) / 2;
        /*
         * [[1.0, 0.0, 0.0],
         * [0.0, 0.7, -0.7],
         * [0.0, 0.7, 0.7]]
         */
        matrix.set(0, 0, 1);
        matrix.set(0, 1, 0);
        matrix.set(0, 2, 0);

        matrix.set(1, 0, 0);
        matrix.set(1, 1, rot);
        matrix.set(1, 2, -rot);

        matrix.set(2, 0, 0);
        matrix.set(2, 1, rot);
        matrix.set(2, 2, rot);

        Rotation3d r = new Rotation3d(matrix);
        assertEquals(Math.PI / 4, r.getX(), DELTA);
        assertEquals(0, r.getY(), DELTA);
        assertEquals(0, r.getZ(), DELTA);
    }

    @Test
    void testPitchMatrix() {
        // in FRC Y is the pitch axis, positive pitch is clockwise looking in the Y
        // direction, which means pitch down.
        Matrix<N3, N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        double rot = Math.sqrt(2) / 2;
        /*
         * [[0.7, 0.0, 0.7],
         * [0.0, 1.0, 0.0],
         * [-0.7, 0.0, 0.7]]
         */
        matrix.set(0, 0, rot);
        matrix.set(0, 1, 0);
        matrix.set(0, 2, rot);

        matrix.set(1, 0, 0);
        matrix.set(1, 1, 1.0);
        matrix.set(1, 2, 0);

        matrix.set(2, 0, -rot);
        matrix.set(2, 1, 0);
        matrix.set(2, 2, rot);

        Rotation3d r = new Rotation3d(matrix);
        assertEquals(0, r.getX(), DELTA);
        assertEquals(Math.PI / 4, r.getY(), DELTA);
        assertEquals(0, r.getZ(), DELTA);
    }

    @Test
    void testYawMatrix() {
        // in FRC Z is the yaw axis, positive yaw is clockwise looking in the Z
        // direction, which means yaw left.
        Matrix<N3, N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        double rot = Math.sqrt(2) / 2;
        /*
         * [[0.7, -0.7, 0.0],
         * [0.7, 0.7, 0.0],
         * [0.0, 0.0, 1.0]]
         */
        matrix.set(0, 0, rot);
        matrix.set(0, 1, -rot);
        matrix.set(0, 2, 0);

        matrix.set(1, 0, rot);
        matrix.set(1, 1, rot);
        matrix.set(1, 2, 0);

        matrix.set(2, 0, 0);
        matrix.set(2, 1, 0);
        matrix.set(2, 2, 1.0);

        Rotation3d r = new Rotation3d(matrix);
        assertEquals(0, r.getX(), DELTA);
        assertEquals(0, r.getY(), DELTA);
        assertEquals(Math.PI / 4, r.getZ(), DELTA);
    }

    @Test
    void testCalibration1() {
        // tag in front of the robot, one meter from center, near the floor.
        Transform3d robotToTag = new Transform3d(new Translation3d(1, 0, 0.3), new Rotation3d());
        // camera is pointing +x (to simplify this test case), offset (+0.3, +0.2, +0.8)
        Transform3d cameraToTag = new Transform3d(new Translation3d(0.7, -0.2, -0.5), new Rotation3d());
        Transform3d robotToCamera = Camera.fromCalibration(robotToTag, cameraToTag);
        assertEquals(0.3, robotToCamera.getX(), DELTA);
        assertEquals(0.2, robotToCamera.getY(), DELTA);
        assertEquals(0.8, robotToCamera.getZ(), DELTA);
        assertEquals(0, robotToCamera.getRotation().getX(), DELTA);
        assertEquals(0, robotToCamera.getRotation().getY(), DELTA);
        assertEquals(0, robotToCamera.getRotation().getZ(), DELTA);
    }

    @Test
    void testCalibration2() {
        // tag in front of the robot, one meter from center, near the floor.
        Transform3d robotToTag = new Transform3d(new Translation3d(1, 0, 0.3), new Rotation3d());
        // camera is pitch down 45, offset (+0.3, 0.0, +0.8)
        Transform3d cameraToTag = new Transform3d(new Translation3d(0.8 * Math.sqrt(2), 0.0, 0.0),
                new Rotation3d(0, -Math.PI / 4, 0));
        Transform3d robotToCamera = Camera.fromCalibration(robotToTag, cameraToTag);
        assertEquals(0.2, robotToCamera.getX(), DELTA);
        assertEquals(0.0, robotToCamera.getY(), DELTA);
        assertEquals(1.1, robotToCamera.getZ(), DELTA);
        assertEquals(0, robotToCamera.getRotation().getX(), DELTA);
        assertEquals(Math.PI / 4, robotToCamera.getRotation().getY(), DELTA);
        assertEquals(0, robotToCamera.getRotation().getZ(), DELTA);
    }

    @Test
    void testCalibration3() {
        // tag on the left (+x) side.
        Transform3d robotToTag = new Transform3d(new Translation3d(0.0, 1.0, 0.3), new Rotation3d(0, 0, Math.PI / 2));
        // camera is pitch down 45, offset (+0.3, 0.0, +0.8)
        Transform3d cameraToTag = new Transform3d(new Translation3d(0.8 * Math.sqrt(2), 0.0, 0.0),
                new Rotation3d(0, -Math.PI / 4, 0));
        Transform3d robotToCamera = Camera.fromCalibration(robotToTag, cameraToTag);
        assertEquals(0.0, robotToCamera.getX(), DELTA);
        assertEquals(0.2, robotToCamera.getY(), DELTA);
        assertEquals(1.1, robotToCamera.getZ(), DELTA);
        assertEquals(0, robotToCamera.getRotation().getX(), DELTA);
        assertEquals(Math.PI / 4, robotToCamera.getRotation().getY(), DELTA);
        assertEquals(Math.PI / 2, robotToCamera.getRotation().getZ(), DELTA);
    }
}
