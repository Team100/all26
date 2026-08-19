package org.team100;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.vision.apriltag.AprilTagDetection;
import org.wpilib.vision.apriltag.AprilTagDetector;
import org.wpilib.vision.apriltag.AprilTagPoseEstimator;
import org.wpilib.vision.camera.OpenCvLoader;

/**
 * Solver for tag poses, in Java.
 * 
 * There are various different methods here; I'm not sure which one is actually
 * correct.
 * 
 * TODO: synthesize a perfect image and use that to choose a method.
 */
public class CornerTest {

    static {
        try {
            OpenCvLoader.forceLoad();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private static final Cv2Util util = new Cv2Util();

    /** Corners from the full-scale image, in case we need them. */
    @SuppressWarnings("unused")
    private static final double[] RAW_CORNERS = new double[] { //
            955, 2477, //
            1934, 1929, //
            1847, 1042, //
            649, 1465 };

    /** Homography from the full-scale image, in case we need it. */
    @SuppressWarnings("unused")
    private static final double[] RAW_HOMOGRAPHY = new double[] { //
            684, 202, 1413, //
            -60, 612, 1741, //
            0, 0, 0 };

    /**
     * Homography between the +/-1 square above and the 0.2-scaled test image.
     */
    private static final double[] EXPECTED_HOMOGRAPHY = new double[] { //
            137, 40, 282, //
            -12, 122, 349, //
            0, 0, 0 };

    /**
     * Corners in in the 0.2-scaled test image.
     */
    private static final double[] EXPECTED_CORNERS = new double[] { //
            191, 496, //
            387, 386, //
            369, 209, //
            130, 293 };

    /**
     * Verify the detection corner locations.
     */
    @Test
    void testDetectionCorners() {
        try (AprilTagDetector detector = new AprilTagDetector()) {
            detector.addFamily("tag36h11");
            Mat img = util.loadVerbatim();
            AprilTagDetection detection = util.getDetection(detector, img);
            double[] corners = detection.getCorners();
            assertArrayEquals(EXPECTED_CORNERS, corners, 1.0);
        }
    }

    /**
     * Verify the detection-derived homography.
     * 
     * Computing the homography is a side-effect of finding the tag.
     */
    @Test
    void testDetectionHomography() {
        try (AprilTagDetector detector = util.detector()) {
            Mat img = util.loadVerbatim();
            AprilTagDetection detection = util.getDetection(detector, img);
            double[] homography = detection.getHomography();
            assertArrayEquals(EXPECTED_HOMOGRAPHY, homography, 1.0);
        }
    }

    /**
     * Verify the OpenCV computation of homography.
     */
    @Test
    void testOpenCvHomography() {
        try (AprilTagDetector detector = util.detector()) {
            Mat img = util.loadVerbatim();
            AprilTagDetection detection = util.getDetection(detector, img);
            double[] corners = detection.getCorners();
            double[] homography = util.getOpenCvHomographyArray(corners);
            assertArrayEquals(EXPECTED_HOMOGRAPHY, homography, 1.0);
        }
    }

    /**
     * The Apriltag library provides access to the older (v1) method of tag pose
     * estimation. It seems worse.
     */
    @Test
    void testDetectedPoseFromHomography() {
        try (AprilTagDetector detector = util.detector()) {
            Mat img = util.loadVerbatim();
            AprilTagDetection detection = util.getDetection(detector, img);
            // double[] corners = detection.getCorners();
            double[] homography = detection.getHomography();
            AprilTagPoseEstimator estimator = util.estimator();
            Transform3d pose = estimator.estimateHomography(homography);
            verifyPose(pose, 0.05, 0.2);
        }
    }

    /**
     * Verify the pose using the Apriltag estimator with the detection corners and
     * homography.
     */
    @Test
    void testDetectedPose() {
        try (AprilTagDetector detector = util.detector()) {
            Mat img = util.loadVerbatim();
            AprilTagDetection detection = util.getDetection(detector, img);
            double[] corners = detection.getCorners();
            double[] homography = detection.getHomography();
            AprilTagPoseEstimator estimator = util.estimator();
            Transform3d pose = estimator.estimate(homography, corners);
            verifyPose(pose, 0.0005, 0.0005);
        }
    }

    /**
     * Use the Apriltag pose estimator with the homography computed by OpenCV, so we
     * don't have to ship the homography from the camera.
     */
    @Test
    void testPoseWithOpenCvHomography() {
        try (AprilTagDetector detector = util.detector()) {
            Mat img = util.loadVerbatim();
            AprilTagDetection detection = util.getDetection(detector, img);
            double[] corners = detection.getCorners();
            double[] homography = util.getOpenCvHomographyArray(corners);
            AprilTagPoseEstimator estimator = util.estimator();
            Transform3d pose = estimator.estimate(homography, corners);
            verifyPose(pose, 0.0005, 0.0005);
        }
    }

    /**
     * The OpenCV "Solve PNP" method.
     * 
     * I think the differences between this and the Apriltag method
     * are caused by the incorrect focal length.
     */
    @Test
    void testOpenCvPose() {
        try (AprilTagDetector detector = util.detector()) {
            Mat img = util.loadVerbatim();
            AprilTagDetection detection = util.getDetection(detector, img);
            double[] corners = detection.getCorners();
            Transform3d pose = util.getOpenCvPose(
                    corners,
                    getCameraMatrix(),
                    new MatOfDouble(util.getDistCoeffs(0)));
            verifyPose(pose, 0.01, 0.3);
        }
    }

    /**
     * Extract corners from a distorted image, undistort them, and verify that
     * they're correct. Note the slightly larger tolerance.
     */
    @Test
    void testCorrectedCorners() {
        final double k1 = -0.3;
        try (AprilTagDetector detector = util.detector()) {
            Mat img = distort(util.loadVerbatim(), k1);
            AprilTagDetection detection = util.getDetection(detector, img);
            double[] corners = getCorrectedCorners(k1, detection);
            assertArrayEquals(EXPECTED_CORNERS, corners, 2.0);
        }
    }

    /**
     * Compute the homography for the corrected corners (using OpenCV) and verify
     * that it is correct.
     */
    @Test
    void testCorrectedHomography() throws IOException {
        try (AprilTagDetector detector = util.detector()) {
            double k1 = -0.3;
            Mat img = distort(util.loadVerbatim(), k1);
            AprilTagDetection detection = util.getDetection(detector, img);
            double[] corners = getCorrectedCorners(k1, detection);
            double[] homography = util.getOpenCvHomographyArray(corners);
            assertArrayEquals(EXPECTED_HOMOGRAPHY, homography, 1.0);
        }
    }

    /**
     * Verify the corrected pose. Note the slightly larger tolerance.
     */
    @Test
    void testDetectionWithDistortion() throws IOException {
        try (AprilTagDetector detector = util.detector()) {
            double k1 = -0.3;
            Mat img = distort(util.loadVerbatim(), k1);
            AprilTagDetection detection = util.getDetection(detector, img);
            double[] corners = getCorrectedCorners(k1, detection);
            double[] homography = util.getOpenCvHomographyArray(corners);
            AprilTagPoseEstimator estimator = util.estimator();
            Transform3d pose = estimator.estimate(homography, corners);
            verifyPose(pose, 0.003, 0.002);
        }
    }

    // This is just to look at the distorted image to make sure it's working.
    // @Test
    void testDetection2() throws IOException {
        Mat distorted_img = distort(util.loadVerbatim(), -0.3);
        Imgcodecs.imwrite("debug.jpg", distorted_img);
    }

    //////////////////////////////////////
    //////////////////////////////////////
    //////////////////////////////////////

    /**
     * Return the corners from the detection, undistorted using k1.
     */
    private double[] getCorrectedCorners(final double k1, AprilTagDetection detection) {
        // the detected corners are now in different places, so we have to fix them.
        double[] detectedCorners = detection.getCorners();
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
                getCameraMatrix(),
                util.getDistCoeffs(k1),
                term);
        Point[] dstL = dstCorners.toArray();
        double[] correctedCorners = new double[] { //
                dstL[0].x, dstL[0].y, //
                dstL[1].x, dstL[1].y, //
                dstL[2].x, dstL[2].y, //
                dstL[3].x, dstL[3].y };
        return correctedCorners;
    }

    /**
     * Use "remap" to invert the undistortion function.
     * 
     * undistortPoints() takes distorted points as input and yields true points.
     * 
     * So we make a matrix of points, each of which is its own location.
     * 
     * When we give undistortPoints() each location, it treats it as if it were from
     * the distorted image, and so yields a location corresponding to the true
     * image. This is, therefore, a "forward" map.
     * 
     * remap() takes each point in the *destination* matrix, and applies the map to
     * find the point in the *source* matrix that should be used. This is an
     * "inverse" map.
     * 
     * So using the "undistort" map with "remap" will invert it, becoming "distort."
     * 
     * Instead, we could apply the analytic distortion model to the input image, but
     * that would require interpolation in the destination (to fill the holes),
     * and that seems harder.
     * 
     * TODO: pull this into lib, make it more like the python one.
     */
    private Mat distort(Mat undistorted_img, double k1) {
        Size size = undistorted_img.size();

        // Compute the map.
        MatOfPoint2f dstPoints = new MatOfPoint2f();
        Calib3d.undistortPoints(
                getSrcPoints(size),
                dstPoints,
                getCameraMatrix(),
                util.getDistCoeffs(k1),
                new Mat(),
                getCameraMatrix());
        List<Point> dstList = dstPoints.toList();

        // Split the map.
        //
        // The remap function takes two maps, one for x and one for y.
        Mat mapX = new Mat(size, CvType.CV_32F);
        Mat mapY = new Mat(size, CvType.CV_32F);
        for (int row = 0; row < size.height; ++row) {
            for (int col = 0; col < size.width; ++col) {
                // row-major
                Point p = dstList.get((int) (size.width) * row + col);
                mapX.put(row, col, p.x);
                mapY.put(row, col, p.y);
            }
        }

        // Apply the map.
        Mat dst = new Mat();
        Imgproc.remap(undistorted_img, dst, mapX, mapY, Imgproc.INTER_LINEAR);
        return dst;
    }

    /** Each src point is just its location (x, y). */
    private MatOfPoint2f getSrcPoints(Size size) {
        List<Point> points = new ArrayList<>();
        for (int row = 0; row < size.height; ++row) {
            for (int col = 0; col < size.width; ++col) {
                points.add(new Point(col, row));
            }
        }
        MatOfPoint2f srcPoints = new MatOfPoint2f();
        srcPoints.fromList(points);
        return srcPoints;
    }

    private Mat getCameraMatrix() {
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        cameraMatrix.put(0, 0, //
                935, 0, 550, //
                0, 935, 310, //
                0, 0, 1);
        return cameraMatrix;
    }

    /**
     * The tag pictured in tag_and_board.jpg is tag #1, and it appears
     * right-side-up.
     * 
     * Guess reasonable pose in camera cordinates.
     * Translation coordinates are (x-right, y-down, z-forward)
     * Rotation coordinates are (x:pitch up, y:yaw right, z:roll clockwise)
     * Translation (-0.25, 0.1, 0.5)
     * Rotation (0.5, -0.4, -0.2)
     */
    private void verifyPose(Transform3d pose, double tTol, double rTol) {
        Translation3d t = pose.getTranslation();
        Rotation3d r = pose.getRotation();
        System.out.printf("T %f %f %f\n", t.getX(), t.getY(), t.getZ());
        System.out.printf("R %f %f %f\n", r.getX(), r.getY(), r.getZ());
        assertEquals(-0.186, t.getX(), tTol);
        assertEquals(0.027, t.getY(), tTol);
        assertEquals(0.642, t.getZ(), tTol);
        assertEquals(0.786, r.getX(), rTol);
        assertEquals(-0.607, r.getY(), rTol);
        assertEquals(-0.492, r.getZ(), rTol);
    }

}
