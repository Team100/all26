package org.team100.lib.gtsam.config;

import gtsam.Cal3DS2;
import gtsam.Point3;
import gtsam.Pose3;
import gtsam.Rot3;

/** See app/config/camera_config.py in all24 */
public class CameraConfig {

    // gtsam uses Pose3 instead of Transform3
    public final Pose3 camera_offset;
    public final Cal3DS2 calib;

    public CameraConfig() throws Throwable {

        camera_offset = new Pose3(
                new Rot3(//
                        0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0),
                new Point3(0, 0, 0.5));
        calib = new Cal3DS2(200.0, 200.0, 0.0, 400.0, 300.0, -0.2, 0.1);
    }
}
