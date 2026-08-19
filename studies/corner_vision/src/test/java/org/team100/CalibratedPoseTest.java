package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.vision.apriltag.AprilTagDetection;
import org.wpilib.vision.apriltag.AprilTagDetector;
import org.wpilib.vision.apriltag.AprilTagPoseEstimator;

public class CalibratedPoseTest {
    private static final Cv2Util util = new Cv2Util();

    /** Try rotating 45 degrees. */
    // @Test
    void testWarp() throws IOException {
        Mat img = util.loadVerbatim2();
        double yrot = Math.PI / 4;
        Mat warped_img = rotate(yrot, img);
        Imgcodecs.imwrite("debug.jpg", warped_img);
    }

    /** Watch debug.jpg to see many angles. */
    // @Test
    void testWarp2() throws IOException, InterruptedException {
        Mat img = util.loadVerbatim2();
        for (double yrot = 0; yrot < Math.PI / 2; yrot += 0.1) {
            Mat warped_img = rotate(yrot, img);
            Imgcodecs.imwrite("debug.jpg", warped_img);
            Thread.sleep(500);
        }
    }

    /**
     * Run the detector on the synthetic image to get the rotation back.
     * 
     * This method works fine.
     * 
     * Note: rotation sensitive to the configured focal length: if you supply a
     * focal length that doesn't match the image, it will struggle to come up with
     * accurate rotation.
     */
    @Test
    void testDetectedPose() {
        try (AprilTagDetector detector = util.detector()) {
            Mat img = util.loadVerbatim2();
            img = rotate(Math.PI / 4, img);
            AprilTagDetection detection = util.getDetection(detector, img);
            double[] corners = detection.getCorners();
            double[] homography = detection.getHomography();
            AprilTagPoseEstimator estimator = util.estimator2();
            Transform3d pose = estimator.estimate(homography, corners);
            verifyPose(pose, 0.007, 0.002);
        }
    }

    /** This yields the same results as above. */
    @Test
    void testDetectedPoseWithOpenCvHomography() {
        try (AprilTagDetector detector = util.detector()) {
            Mat img = util.loadVerbatim2();
            img = rotate(Math.PI / 4, img);
            AprilTagDetection detection = util.getDetection(detector, img);
            double[] corners = detection.getCorners();
            double[] homography = util.getOpenCvHomographyArray(corners);
            AprilTagPoseEstimator estimator = util.estimator2();
            Transform3d pose = estimator.estimate(homography, corners);
            verifyPose(pose, 0.007, 0.002);
        }
    }

    /**
     * The OpenCV way.
     * 
     * This yields exactly the same accuracy as above.
     */
    @Test
    void testOpenCvPose() {
        try (AprilTagDetector detector = util.detector()) {
            Mat img = util.loadVerbatim2();
            img = rotate(Math.PI / 4, img);
            AprilTagDetection detection = util.getDetection(detector, img);
            double[] corners = detection.getCorners();
            Transform3d pose = util.getOpenCvPose(
                    corners,
                    getCameraMatrix(),
                    new MatOfDouble(util.getDistCoeffs(0)));
            verifyPose(pose, 0.007, 0.002);
        }
    }

    ////////////////////////////////////
    ////////////////////////////////////
    ////////////////////////////////////

    private Mat getCameraMatrix() {
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        cameraMatrix.put(0, 0, //
                1500, 0, 704, //
                0, 1500, 544, //
                0, 0, 1);
        return cameraMatrix;
    }

    private void verifyPose(Transform3d pose, double tTol, double rTol) {
        Translation3d t = pose.getTranslation();
        Rotation3d r = pose.getRotation();
        System.out.printf("T %f %f %f\n", t.getX(), t.getY(), t.getZ());
        System.out.printf("R %f %f %f\n", r.getX(), r.getY(), r.getZ());
        assertEquals(0, t.getX(), tTol);
        assertEquals(0, t.getY(), tTol);
        // this is a function of the tag size in the image and f
        assertEquals(0.424, t.getZ(), tTol);
        assertEquals(0, r.getX(), rTol);
        assertEquals(Math.PI / 4, r.getY(), rTol);
        assertEquals(0, r.getZ(), rTol);
    }

    private Mat rotate(double yrot, Mat img) {
        Size size = img.size();
        double cx = size.width / 2;
        double cy = size.height / 2;
        Mat warped_img = new Mat();
        double f = 1500;

        // to 3d
        Mat A1 = new Mat(4, 3, CvType.CV_64FC1);
        A1.put(0, 0, //
                1, 0, -cx, //
                0, 1, -cy, //
                0, 0, 0, //
                0, 0, 1);
        // transform is yaw around y, and moving ahead in z
        Mat T = new Mat(4, 4, CvType.CV_64FC1);
        T.put(0, 0, //
                Math.cos(yrot), 0, Math.sin(yrot), 0, //
                0, 1, 0, 0, //
                -Math.sin(yrot), 0, Math.cos(yrot), f, //
                0, 0, 0, 1);
        // projection
        Mat A2 = new Mat(3, 4, CvType.CV_64FC1);
        A2.put(0, 0, //
                f, 0, cx, 0, //
                0, f, cy, 0, //
                0, 0, 1, 0);
        Mat M = A2.matMul(T).matMul(A1);
        // System.out.printf("M\n%s\n", M.dump());
        Imgproc.warpPerspective(img, warped_img, M, img.size(),
                Imgproc.INTER_LINEAR, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));
        return warped_img;
    }
}
