package org.team100.lib.camera;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * For fixed cameras, the offset from the robot zero (center of the frame, on
 * the floor, x-forward, z-up) to the camera pose (x-forward, z-up)
 */
public class Offset {
    private static final Offset DEFAULT = new Offset(new Transform3d());
    private static final Map<Camera, Offset> offsets;

    static {
        offsets = new EnumMap<>(Camera.class);
        offsets.put(Camera.CAMERA_BACK, new Offset(
                fromCalibration(
                        new Transform3d(0.96, 0.031, 0.57, new Rotation3d(0, 0, Math.PI)),
                        new Transform3d(0.926, -0.293, -0.165, new Rotation3d(1.599, 0.03, 0.09)))));
        offsets.put(Camera.CAMERA_FRONT, new Offset(
                fromCalibration(
                        new Transform3d(0.96, 0.031, 0.57, new Rotation3d(0, 0, 0)),
                        new Transform3d(1.001, 0.267, -0.044, new Rotation3d(-1.57, -0.02, -0.05)))));
        offsets.put(Camera.C, new Offset(
                new Transform3d(new Translation3d(0, 0, 1), new Rotation3d(0, -Math.toRadians(10), 0))));
        offsets.put(Camera.RIGHTAMP, new Offset(
                new Transform3d(
                        new Translation3d(-0.1265, -0.1063625, 0.61),
                        new Rotation3d(0, Math.toRadians(-26), Math.toRadians(-63)))));
        offsets.put(Camera.LEFTAMP, new Offset(
                new Transform3d(
                        new Translation3d(-0.1265, 0.1532, 0.61),
                        new Rotation3d(0, Math.toRadians(-22), Math.toRadians(59)))));
        offsets.put(Camera.GAME_PIECE, new Offset(
                new Transform3d(
                        new Translation3d(-0.1265, 0.03, 0.61),
                        new Rotation3d(0, Math.toRadians(31.5), Math.PI))));
        offsets.put(Camera.SWERVE_RIGHT, new Offset(
                new Transform3d(
                        new Translation3d(-0.261, -0.317, 0.217),
                        new Rotation3d(-0.146, 0.195, -0.508).unaryMinus().plus(new Rotation3d(0, 0, -Math.PI / 2)))));
        offsets.put(Camera.SWERVE_LEFT, new Offset(
                new Transform3d(
                        new Translation3d(-0.241, 0.297, 0.207),
                        new Rotation3d(0.07, 0.147, 0.52).unaryMinus().plus(new Rotation3d(0, 0, Math.PI / 2)))));
        offsets.put(Camera.FUNNEL, new Offset(
                new Transform3d(
                        new Translation3d(-0.034, -0.213, 0.902),
                        new Rotation3d(0.07, 0.48, 0.20).unaryMinus().plus(new Rotation3d(0, 0, Math.PI)))));
        offsets.put(Camera.CLIMB_LEFT, new Offset(
                fromCalibration(new Transform3d(0.34, 1, 0.398, new Rotation3d(0, 0, Math.PI / 2)),
                        new Transform3d(0.84, -0.28, -0.19, new Rotation3d(0.005, 0.475, -0.042)))));
        offsets.put(Camera.CLIMB_RIGHT, new Offset(
                fromCalibration(
                        new Transform3d(-0.135, 0.843, 0.432, new Rotation3d(0, 0, Math.PI / 2)),
                        new Transform3d(0.563, -0.068, -0.0356, new Rotation3d(-0.087, 0.529, -0.172)))));
        offsets.put(Camera.SHOOTER, new Offset(
                fromCalibration(
                        new Transform3d(0.34, -1, 0.405, new Rotation3d(0, 0, -(Math.PI / 2))),
                        new Transform3d(0.812, 0.192, -0.354, new Rotation3d(-0.016, 0.630, -0.040)))));

        //////////////////////////////////////
        //
        // For unit tests.
        //
        offsets.put(Camera.ORIGIN, new Offset(
                new Transform3d()));
        offsets.put(Camera.TEST4, new Offset(
                new Transform3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, 0))));
        offsets.put(Camera.TEST5, new Offset(
                new Transform3d(
                        new Translation3d(0, 0.1, 1),
                        new Rotation3d(0, 0, 0))));
        offsets.put(Camera.TEST6, new Offset(
                new Transform3d(
                        new Translation3d(0.198, 0.284, 0.811),
                        new Rotation3d(-0.043, -0.705, 0.254).unaryMinus())));
        offsets.put(Camera.TEST7, new Offset(
                new Transform3d(
                        new Translation3d(1, 0, 1.368),
                        new Rotation3d(0, -0.523, 0))));
        offsets.put(Camera.TEST7A, new Offset(
                new Transform3d(
                        new Translation3d(0, 0, 1.368),
                        new Rotation3d(0, -0.523, 0))));
        offsets.put(Camera.TEST8, new Offset(
                new Transform3d(
                        new Translation3d(),
                        new Rotation3d(0, Math.PI / 4, 0))));
        offsets.put(Camera.TEST9, new Offset(
                new Transform3d(
                        new Translation3d(),
                        new Rotation3d(0, Math.PI / 6, 0))));
        offsets.put(Camera.SIM0, new Offset(
                new Transform3d(
                        new Translation3d(0, 0, 0.75),
                        new Rotation3d(0, 0, 0))));
        offsets.put(Camera.SIM1, new Offset(
                new Transform3d(
                        new Translation3d(0, 0, 0.75),
                        new Rotation3d(0, 0, Math.PI / 2))));
        offsets.put(Camera.SIM2, new Offset(
                new Transform3d(
                        new Translation3d(0, 0, 0.75),
                        new Rotation3d(0, 0, Math.PI))));
        offsets.put(Camera.SIM3, new Offset(
                new Transform3d(
                        new Translation3d(0, 0, 0.75),
                        new Rotation3d(0, 0, -Math.PI / 2))));
        offsets.put(Camera.UNKNOWN,
                new Offset(new Transform3d()));
    }

    public static Offset get(Camera camera) {
        Offset o = offsets.get(camera);
        if (o == null) {
            System.out.printf("Unknown offset for camera %s\n", camera);
            return DEFAULT;
        }
        return o;
    }

    private final Transform3d offset;

    public Offset(Transform3d offset) {
        this.offset = offset;
    }

    public Transform3d offset() {
        return offset;
    }

    /**
     * Use this to calibrate the cameras. Set the transform to identity, set a tag
     * in a known location ("robot to tag"), and enter what the camera thinks the
     * tag pose is -- this appears on glass as "camera to tag".
     */
    static Transform3d fromCalibration(Transform3d robotToTag, Transform3d cameraToTag) {
        return robotToTag.plus(cameraToTag.inverse());
    }
}
