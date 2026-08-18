package org.team100.lib.localization;

import java.io.IOException;
import java.util.EnumMap;
import java.util.Map;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.TermCriteria;
import org.team100.lib.camera.Camera;
import org.team100.lib.camera.Distortion;
import org.team100.lib.camera.Intrinsic;

import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Use the AprilTag corners to derive a pose estimate.
 * 
 * This is a half-step to GTSAM; it's not better than
 * doing the pose estimation on the Raspberry Pi.
 */
public class PoseFromCorners {
    private static final double TAG_SIZE_M = 0.1651;

    static {
        try {
            OpenCvLoader.forceLoad();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    private static final MatOfPoint2f CORNERS_FOR_HOMOGRAPHY = new MatOfPoint2f(
            new Point(-1, 1),
            new Point(1, 1),
            new Point(1, -1),
            new Point(-1, -1));

    private final Map<Camera, AprilTagPoseEstimator> estimators;

    public PoseFromCorners() {
        estimators = new EnumMap<>(Camera.class);
    }

    /**
     * Compute the pose based on the corners, with distortion correction.
     */
    public Transform3d pose(Camera camera, double[] detectedCorners) {
        double[] corners = correctedCorners(camera, detectedCorners);
        double[] homography = getOpenCvHomographyArray(corners);
        AprilTagPoseEstimator estimator = estimators.computeIfAbsent(
                camera, this::makeEstimator);
        return estimator.estimate(homography, corners);
    }

    private AprilTagPoseEstimator makeEstimator(Camera camera) {
        Intrinsic i = Intrinsic.get(camera);
        AprilTagPoseEstimator.Config conf = new AprilTagPoseEstimator.Config(
                TAG_SIZE_M, i.fx(), i.fy(), i.cx(), i.cy());
        return new AprilTagPoseEstimator(conf);
    }

    private double[] getOpenCvHomographyArray(double[] corners) {
        MatOfPoint2f dstPoints = new MatOfPoint2f(
                new Point(corners[0], corners[1]),
                new Point(corners[2], corners[3]),
                new Point(corners[4], corners[5]),
                new Point(corners[6], corners[7]));
        Mat openCvHomographyMat = Calib3d.findHomography(
                CORNERS_FOR_HOMOGRAPHY, dstPoints);
        double[] openCvHomographyArray = new double[9];
        openCvHomographyMat.get(0, 0, openCvHomographyArray);
        return openCvHomographyArray;
    }

    private double[] correctedCorners(Camera camera, double[] detectedCorners) {
        Intrinsic i = Intrinsic.get(camera);
        Distortion d = Distortion.get(camera);

        MatOfPoint2f srcCorners = new MatOfPoint2f(
                new Point(detectedCorners[0], detectedCorners[1]),
                new Point(detectedCorners[2], detectedCorners[3]),
                new Point(detectedCorners[4], detectedCorners[5]),
                new Point(detectedCorners[6], detectedCorners[7]));
        MatOfPoint2f dstCorners = new MatOfPoint2f();

        // Undistort the corner points.
        //
        // Add extra iterations to be sure? This seems not to matter for this
        // particular case but it's not a bad idea in general.
        TermCriteria term = new TermCriteria(
                TermCriteria.COUNT | TermCriteria.EPS, 40, 0.01);
        Calib3d.undistortImagePoints(
                srcCorners,
                dstCorners,
                i.mat(),
                d.mat(),
                term);
        Point[] dstL = dstCorners.toArray();
        double[] correctedCorners = new double[] { //
                dstL[0].x, dstL[0].y, //
                dstL[1].x, dstL[1].y, //
                dstL[2].x, dstL[2].y, //
                dstL[3].x, dstL[3].y };
        return correctedCorners;
    }

}
