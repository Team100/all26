package org.team100.lib.localization.gtsam;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.camera.Camera;
import org.team100.lib.camera.Distortion;
import org.team100.lib.camera.Intrinsic;
import org.team100.lib.camera.Offset;
import org.team100.lib.localization.map.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.network.BlipWithCorners;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

public class TagToCorners {
    private static final double TAG_SIZE_M = 0.1651;
    private static final double H = TAG_SIZE_M / 2;
    private static final Transform3d LL = new Transform3d(0, H, -H, Rotation3d.kZero);
    private static final Transform3d LR = new Transform3d(0, -H, -H, Rotation3d.kZero);
    private static final Transform3d UR = new Transform3d(0, -H, H, Rotation3d.kZero);
    private static final Transform3d UL = new Transform3d(0, H, H, Rotation3d.kZero);

    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    private final Supplier<Optional<Alliance>> m_alliance;

    public TagToCorners(Supplier<Optional<Alliance>> alliance,
            AprilTagFieldLayoutWithCorrectOrientation layout) {
        m_layout = layout;
        m_alliance = alliance;
    }

    public List<Corners> convert(Camera camera, BlipWithCorners[] blips) {
        Transform3d offset = Offset.get(camera).offset();
        Intrinsic intrinsic = Intrinsic.get(camera);
        Distortion distortion = Distortion.get(camera);

        // Fetch the alliance (not available immediately after startup).
        Optional<Alliance> optAlliance = m_alliance.get();
        if (!optAlliance.isPresent()) {
            return List.of();
        }
        Alliance alliance = optAlliance.get();
        List<Corners> corners = new ArrayList<>();
        for (int i = 0; i < blips.length; ++i) {
            BlipWithCorners blip = blips[i];

            Optional<Pose3d> tagInFieldOpt = m_layout.getTagPose(alliance, blip.getId());
            if (!tagInFieldOpt.isPresent()) {
                // This shouldn't happen, but it does.
                System.out.printf("WARNING: VisionDataProvider24: no tag for id %d\n", blip.getId());
                continue;
            }

            Pose3d tagInField = tagInFieldOpt.get();

            double blipTimeSec = (double) blip.getTimestamp() / 1e6;

            // lower-left
            corners.add(new Corners(
                    blipTimeSec,
                    tagInField.transformBy(LL).getTranslation(),
                    new Translation2d(blip.getX0(), blip.getY0()),
                    offset,
                    intrinsic,
                    distortion));
            // lower-right
            corners.add(new Corners(
                    blipTimeSec,
                    tagInField.transformBy(LR).getTranslation(),
                    new Translation2d(blip.getX1(), blip.getY1()),
                    offset,
                    intrinsic,
                    distortion));
            // upper-right
            corners.add(new Corners(
                    blipTimeSec,
                    tagInField.transformBy(UR).getTranslation(),
                    new Translation2d(blip.getX2(), blip.getY2()),
                    offset,
                    intrinsic,
                    distortion));
            // upper-left
            corners.add(new Corners(
                    blipTimeSec,
                    tagInField.transformBy(UL).getTranslation(),
                    new Translation2d(blip.getX3(), blip.getY3()),
                    offset,
                    intrinsic,
                    distortion));
        }
        return corners;
    }

}
