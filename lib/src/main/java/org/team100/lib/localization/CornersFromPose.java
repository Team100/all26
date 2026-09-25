package org.team100.lib.localization;

import java.io.IOException;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.team100.lib.camera.Camera;
import org.team100.lib.camera.Distortion;
import org.team100.lib.camera.Intrinsic;
import org.team100.lib.geometry.OpenCVUtil;

import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Project AprilTag corners into (u,v) camera coordinates, for testing.
 */
public class CornersFromPose {
    private static final double TAG_SIZE_M = 0.1651;
    private static final double h = TAG_SIZE_M / 2;

    static {
        try {
            OpenCvLoader.forceLoad();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    private final Transform3d corner0 = new Transform3d(-h, h, 0, Rotation3d.kZero);
    private final Transform3d corner1 = new Transform3d(h, h, 0, Rotation3d.kZero);
    private final Transform3d corner2 = new Transform3d(h, -h, 0, Rotation3d.kZero);
    private final Transform3d corner3 = new Transform3d(-h, -h, 0, Rotation3d.kZero);

    /**
     * Project the corners of the tag into the camera.
     * 
     * @param tagInCamera z-forward, camera coordinates.
     * @return four (u,v) camera pixel pairs, starting in lower left, then
     *         counter-clockwise.
     */
    public double[] corners(Camera camera, Transform3d tagInCamera) {
        Transform3d c0 = tagInCamera.plus(corner0);
        Transform3d c1 = tagInCamera.plus(corner1);
        Transform3d c2 = tagInCamera.plus(corner2);
        Transform3d c3 = tagInCamera.plus(corner3);
        MatOfPoint3f objectPts = new MatOfPoint3f(
                OpenCVUtil.point(c0.getTranslation()),
                OpenCVUtil.point(c1.getTranslation()),
                OpenCVUtil.point(c2.getTranslation()),
                OpenCVUtil.point(c3.getTranslation()));
        Mat rvec = Mat.zeros(3, 1, CvType.CV_64F);
        Mat tvec = Mat.zeros(3, 1, CvType.CV_64F);
        Mat kMat = Intrinsic.get(camera).mat();
        MatOfDouble dMat = Distortion.get(camera).matOfDouble();
        MatOfPoint2f out = new MatOfPoint2f();
        Calib3d.projectPoints(objectPts, rvec, tvec, kMat, dMat, out);
        Point[] pts = out.toArray();
        return new double[] {
                pts[0].x, pts[0].y,
                pts[1].x, pts[1].y,
                pts[2].x, pts[2].y,
                pts[3].x, pts[3].y
        };
    }

}
