package org.team100.lib.gtsam.simulation;

import java.util.List;
import java.util.Random;

import org.wpilib.math.geometry.Pose2d;

import org.team100.lib.gtsam.config.CameraConfig;
import gtsam.Cal3DS2;
import gtsam.PinholeCamera;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose2;
import gtsam.Pose3;
import org.team100.lib.gtsam.util.Geometry;

/**
 * Simulated camera measurements for a single tag.
 */
public class SimulatedCamera {
    private static Random RANDOM = new Random(42);
    private final List<Point3> landmarks;
    private final Pose3 camera_offset;
    private final Cal3DS2 calib;

    public SimulatedCamera(List<Point3> landmarks, CameraConfig cam) throws Throwable {
        this.landmarks = landmarks;
        camera_offset = cam.camera_offset;
        calib = cam.calib;
    }

    /**
     * Given a robot pose, project tag corners into pixel measurements.
     * 
     * @param gtPose2d robot pose
     * @return list of pixels corresponding to tag corners
     */
    public List<Point2> pixels(Pose2d gtPose2d) throws Throwable {
        final Pose2 robot_pose = Geometry.toPose2(gtPose2d);
        // lower left
        Point2 p0 = pixel(landmarks.get(0), robot_pose);
        // lower right
        Point2 p1 = pixel(landmarks.get(1), robot_pose);
        // upper right
        Point2 p2 = pixel(landmarks.get(2), robot_pose);
        // upper left
        Point2 p3 = pixel(landmarks.get(3), robot_pose);
        List<Point2> gt_pixels = List.of(p0, p1, p2, p3);
        // Omit out-of-frame tags.
        for (Point2 p : gt_pixels) {
            double x = p.x();
            double y = p.y();
            if (x < 0 || y < 0 || x > 800 || y > 600) {
                // any corner out of frame means the whole tag is not seen
                return List.of();
            }
        }
        return gt_pixels;
    }

    /**
     * Project the landmark point into the camera frame and return (x, y) in pixels.
     * Robot_pose and camera_offset are x-forward, z-up.
     * 
     * Includes one-pixel uniform noise.
     */
    private Point2 pixel(Point3 landmark, Pose2 robot_pose) throws Throwable {
        Pose3 camera_pose = new Pose3(robot_pose).compose(camera_offset);
        PinholeCamera<Cal3DS2> camera = PinholeCamera.PinholeCameraCal3DS2(
                camera_pose, calib);
        Point2 pxNoise = noise();
        return camera.project(landmark).plus(pxNoise);
    }

    private Point2 noise() throws Throwable {
        return zeroNoise();
        // return uniformNoise();
    }

    private Point2 zeroNoise() throws Throwable {
        return new Point2(0, 0);
    }

    private Point2 uniformNoise() throws Throwable {
        return new Point2(uniform(1), uniform(1));
    }

    private double uniform(double a) {
        return RANDOM.nextDouble(-a, a);
    }

}
