package org.team100.lib.camera;

import java.util.EnumMap;
import java.util.Map;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

/**
 * Camera distortion parameters.
 */
public class Distortion {
    private static final Distortion DEFAULT = new Distortion(
            new double[] { 0.0, 0.0, 0.0, 0.0 });
    private static final Map<Camera, Distortion> distortions;

    static {
        distortions = new EnumMap<>(Camera.class);
        distortions.put(Camera.SIM0, DEFAULT);
        distortions.put(Camera.SIM1, DEFAULT);
        distortions.put(Camera.SIM2, DEFAULT);
        distortions.put(Camera.SIM3, DEFAULT);
        // TODO: more cameras
    }

    public static Distortion get(Camera camera) {
        Distortion d = distortions.get(camera);
        if (d == null) {
            System.out.printf("Unknown distortion for camera %s\n", camera);
            return DEFAULT;
        }
        return d;
    }

    private final double[] dist;
    private final Mat distMat;
    private final MatOfDouble distMatOfDouble;

    public Distortion(double[] dist) {
        this.dist = dist;
        this.distMat = new Mat(1, 4, CvType.CV_64FC1);
        distMat.put(0, 0, dist[0], dist[1], dist[2], dist[3]);
        this.distMatOfDouble = new MatOfDouble(dist);
    }

    public Mat mat() {
        return distMat;
    }

    public MatOfDouble matOfDouble() {
        return distMatOfDouble;
    }

    public double[] dist() {
        return dist;
    }

    public double k1() {
        return dist[0];
    }

    public double k2() {
        return dist[1];
    }

    public double p1() {
        return dist[2];
    }

    public double p2() {
        return dist[3];
    }

}
