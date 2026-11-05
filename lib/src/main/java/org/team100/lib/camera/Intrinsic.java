package org.team100.lib.camera;

import java.util.EnumMap;
import java.util.Map;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * Camera intrinsic matrices.
 * 
 * Should mirror intrinsic.py.
 */
public class Intrinsic {
    private static final Intrinsic DEFAULT = new Intrinsic(new double[][] { //
            { 1.0, 0.0, 0.0 }, //
            { 0.0, 1.0, 0.0 }, //
            { 0.0, 0.0, 1.0 } });
    private static final Intrinsic SIM_DEFAULT = new Intrinsic(new double[][] { //
            { 935.0, 0.0, 550.0 }, //
            { 0.0, 935.0, 550.0 }, //
            { 0.0, 0.0, 1.0 } });
    private static final Map<Camera, Intrinsic> intrinsics;

    static {
        intrinsics = new EnumMap<>(Camera.class);
        intrinsics.put(Camera.SIM0, SIM_DEFAULT);
        intrinsics.put(Camera.SIM1, SIM_DEFAULT);
        intrinsics.put(Camera.SIM2, SIM_DEFAULT);
        intrinsics.put(Camera.SIM3, SIM_DEFAULT);
        // TODO: more cameras
    }

    public static Intrinsic get(Camera camera) {
        Intrinsic i = intrinsics.get(camera);
        if (i == null) {
            System.out.printf("Unknown intrinsic for camera %s\n", camera);
            return DEFAULT;
        }
        return i;
    }

    private final double[][] K;
    private final Mat Kmat;

    public Intrinsic(double[][] K) {
        this.K = K;
        this.Kmat = new Mat(3, 3, CvType.CV_32FC1);
        Kmat.put(0, 0, //
                K[0][0], K[0][1], K[0][2], //
                K[1][0], K[1][1], K[1][2], //
                K[2][0], K[2][1], K[2][2]);
    }

    public Mat mat() {
        return Kmat;
    }

    public double fx() {
        return K[0][0];
    }

    public double fy() {
        return K[1][1];
    }

    public double cx() {
        return K[0][2];
    }

    public double cy() {
        return K[1][2];
    }

}
