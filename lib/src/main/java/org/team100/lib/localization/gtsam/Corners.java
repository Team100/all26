package org.team100.lib.localization.gtsam;

import org.team100.lib.camera.Distortion;
import org.team100.lib.camera.Intrinsic;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The camera offset here uses x-forward coordinates not camera coordinates.
 */
public record Corners(
        double timestamp,
        Translation3d landmark,
        Translation2d measurement,
        Transform3d offset,
        Intrinsic intrinsic,
        Distortion distortion) {

}
