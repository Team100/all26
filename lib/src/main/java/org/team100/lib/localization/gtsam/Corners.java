package org.team100.lib.localization.gtsam;

import org.team100.lib.camera.Distortion;
import org.team100.lib.camera.Intrinsic;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

public record Corners(
        double timestamp,
        Pose3d tag,
        Translation2d corner,
        Transform3d offset,
        Intrinsic intrinsic,
        Distortion distortion) {

}
