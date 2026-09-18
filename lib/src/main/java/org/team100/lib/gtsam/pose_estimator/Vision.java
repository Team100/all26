package org.team100.lib.gtsam.pose_estimator;

import org.team100.lib.gtsam.config.CameraConfig;
import gtsam.Cal3DS2;
import gtsam.Key;
import gtsam.PlanarProjectionFactor1;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose3;
import gtsam.Vector2;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

/**
 * Use PlanarProjectionFactor to model camera measurements.
 */
public class Vision {
    private final Solver estimate;
    private final Pose3 camera_offset;
    private final Cal3DS2 calib;
    private final shared_ptr<Diagonal> noise;

    public Vision(Solver e, CameraConfig conf) throws Throwable {
        estimate = e;
        camera_offset = conf.camera_offset;
        calib = conf.calib;
        // pixel noise is small. mistakes in calibration will result
        // in larger errors, though.
        noise = Diagonal.Sigmas(new Vector2(1, 1));
    }

    /**
     * Add a factor for the measurement of the landmark.
     */
    public void add(long t1_us, Point3 landmark, Point2 measurement)
            throws Throwable {
        estimate.add(PlanarProjectionFactor1.newPlanarProjectionFactor1(
                Key.X(t1_us),
                landmark,
                measurement,
                camera_offset,
                calib,
                noise));
    }

}
