package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.io.IOException;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.vision.apriltag.AprilTagDetection;
import org.wpilib.vision.apriltag.AprilTagDetector;
import org.wpilib.vision.apriltag.AprilTagPoseEstimator;
import org.wpilib.vision.camera.OpenCvLoader;

public class Cv2Util {
    static {
        try {
            OpenCvLoader.forceLoad();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * The tag coordinate system defines the corner order, which starts in the lower
     * left, and goes counter-clockwise, using the camera convention (x-right,
     * y-down).
     */
    private static final MatOfPoint2f CORNERS_FOR_HOMOGRAPHY = new MatOfPoint2f(
            new Point(-1, 1),
            new Point(1, 1),
            new Point(1, -1),
            new Point(-1, -1));

    /**
     * Find files in the classpath.
     * 
     * Test resources should go in src/test/resources,
     * and they appear in build/resources/test.
     */
    private String res(String filename) {
        // this puts "/" before "C:" on windows which does not work
        return getClass().getResource("/" + filename).getPath();
    }

    /**
     * Load the test image and scale it to 0.2 in each dimension.
     */
    Mat loadVerbatim() {
        // TODO: fix this filename issue
        Mat img = Imgcodecs.imread(res("tag_and_board.jpg"));
        // Mat img = Imgcodecs.imread(
        // "C:/Users/joel/FRC/TRUHER/all26/studies/corner_vision/build/resources/test/tag_and_board.jpg");
        assertNotNull(img);
        Size size = img.size();
        assertEquals(5504, size.width);
        assertEquals(3096, size.height);
        // the equivalent tag_detector_test.py resizes, so we do too.
        Imgproc.resize(img, img, new Size(1100, 620));
        size = img.size();
        assertEquals(1100, size.width);
        assertEquals(620, size.height);
        return img;
    }

    Mat loadVerbatim2() {
        // TODO: fix this filename issue
        Mat img = Imgcodecs.imread(res("big_sharp.png"));
        // Mat img = Imgcodecs.imread(
        // "C:/Users/joel/FRC/TRUHER/all26/studies/corner_vision/build/resources/test/tag_and_board.jpg");
        assertNotNull(img);
        Size size = img.size();
        assertEquals(1408, size.width);
        assertEquals(1088, size.height);
        return img;
    }

    /**
     * Make a detector looking for 36h11. Remember to close it.
     */
    AprilTagDetector detector() {
        AprilTagDetector detector = new AprilTagDetector();
        detector.addFamily("tag36h11");
        return detector;
    }

    /**
     * The pose estimator uses the real tag size, and the same intrinsics as the
     * python test, which is similar to the Raspberry Pi GS camera.
     */
    AprilTagPoseEstimator estimator() {
        AprilTagPoseEstimator.Config conf = new AprilTagPoseEstimator.Config(
                0.1651, 935, 935, 550, 310);
        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(conf);
        return estimator;
    }

    /** For the image fetched by loadVerbatim2 */
    AprilTagPoseEstimator estimator2() {
        AprilTagPoseEstimator.Config conf = new AprilTagPoseEstimator.Config(
                0.1651, 1500, 1500, 704, 544);
        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(conf);
        return estimator;
    }

    AprilTagDetection getDetection(AprilTagDetector detector, Mat img) {
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2GRAY);
        AprilTagDetection[] detections = detector.detect(img);
        // we find the big one
        assertEquals(1, detections.length);
        AprilTagDetection detection = detections[0];
        assertEquals(1, detection.getId());
        return detection;
    }

    double[] getOpenCvHomographyArray(double[] corners) {
        MatOfPoint2f dstPoints = new MatOfPoint2f(
                new Point(corners[0], corners[1]),
                new Point(corners[2], corners[3]),
                new Point(corners[4], corners[5]),
                new Point(corners[6], corners[7]));
        Mat openCvHomographyMat = Calib3d.findHomography(CORNERS_FOR_HOMOGRAPHY, dstPoints);
        double[] openCvHomographyArray = new double[9];
        openCvHomographyMat.get(0, 0, openCvHomographyArray);
        return openCvHomographyArray;
    }

    /**
     * Extract the pose using openCV.
     * This sucks, the Apriltag one is better.
     */
    Transform3d getOpenCvPose(double[] detectedCorners, Mat mtx, MatOfDouble dist) {
        MatOfPoint3f obj = new MatOfPoint3f(
                new Point3(-0.08255, 0.08255, 0.0),
                new Point3(0.08255, 0.08255, 0.0),
                new Point3(0.08255, -0.08255, 0.0),
                new Point3(-0.08255, -0.08255, 0.0));
        MatOfPoint2f dstPoints = new MatOfPoint2f(
                new Point(detectedCorners[0], detectedCorners[1]),
                new Point(detectedCorners[2], detectedCorners[3]),
                new Point(detectedCorners[4], detectedCorners[5]),
                new Point(detectedCorners[6], detectedCorners[7]));
        return pnp(mtx, dist, obj, dstPoints);
    }

    private Transform3d pnp(Mat mtx, MatOfDouble dist,
            MatOfPoint3f obj, MatOfPoint2f dstPoints) {
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        Calib3d.solvePnP(obj, dstPoints, mtx, dist, rvec, tvec,
                false, Calib3d.SOLVEPNP_IPPE_SQUARE);
        Transform3d openCvPose = new Transform3d(
                new Translation3d(tvec.get(0, 0)[0], tvec.get(1, 0)[0], tvec.get(2, 0)[0]),
                new Rotation3d(rvec.get(0, 0)[0], rvec.get(1, 0)[0], rvec.get(2, 0)[0]));
        return openCvPose;
    }

    // RANSAC seems worse for this case.
    @SuppressWarnings("unused")
    private Transform3d pnpRansac(
            Mat mtx, MatOfDouble dist, MatOfPoint3f obj,
            MatOfPoint2f dstPoints) {
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        Calib3d.solvePnPRansac(obj, dstPoints, mtx, dist, rvec, tvec,
                false, 100, (float) 0.1, 0.99, new Mat(),
                Calib3d.SOLVEPNP_IPPE_SQUARE);
        Transform3d openCvPose = new Transform3d(
                new Translation3d(tvec.get(0, 0)[0], tvec.get(1, 0)[0], tvec.get(2, 0)[0]),
                new Rotation3d(rvec.get(0, 0)[0], rvec.get(1, 0)[0], rvec.get(2, 0)[0]));
        return openCvPose;
    }

    Mat getDistCoeffs(double k1) {
        Mat distCoeffs = new Mat(1, 4, CvType.CV_64FC1);
        distCoeffs.put(0, 0, k1, 0.0, 0.0, 0.0);
        return distCoeffs;
    }

}
