package org.team100.lib.localization.gtsam;

import java.util.Collection;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.Supplier;

import org.team100.lib.camera.Distortion;
import org.team100.lib.camera.Intrinsic;
import org.team100.lib.gtsam.util.Geometry;
import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.util.Math100;
import org.team100.lib.util.TimeInterpolatableBuffer100;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.util.Nat;

import gtsam.Cal3DS2;
import gtsam.FixedLagSmoother.KeyTimestampMap;
import gtsam.Key;
import gtsam.NonlinearFactorGraph;
import gtsam.PlanarGyroFactor;
import gtsam.PlanarGyroFactor.PlanarGyroBiasFactor;
import gtsam.PlanarGyroFactor.PlanarGyroParams;
import gtsam.PlanarProjectionFactor1;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose2;
import gtsam.Pose3;
import gtsam.Rot2;
import gtsam.Values;
import gtsam.Vector2;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

/**
 * TODO: remove the odometry stuff from this class, do it elsewhere.
 */
public class Vision {

    /**
     * The camera coordinates is z-forward, x-right, but the camera
     * "offset" is measured in x-forward, z-up coordinates (because
     * it's simpler to think about). Add this transform to get
     * the actual offset, with the z-axis pointing in the right
     * direction.
     */
    static final Transform3d CAM = new Transform3d(
            Translation3d.kZero,
            new Rotation3d(MatBuilder.fill(Nat.N3(), Nat.N3(), //
                    0, 0, 1, //
                    -1, 0, 0, //
                    0, -1, 0)));

    // the camera itself is good to a pixel; the apriltag detector
    // might be good to a few pixels, depending on the lens, and the
    // lens calibration might be good to a pixel.
    // TODO: look carefully at camera accuracy
    static final shared_ptr<Diagonal> PX_NOISE = Diagonal.Sigmas(new Vector2(2, 2));

    /**
     * TODO: move the constants here to the class, make this not static.
     * 
     * @return the list of timestamps that need pose values, timestamps,
     *         prior factors, and between factors (if possible).
     */
    static Collection<Double> vision(
            List<Corners> corners,
            NonlinearFactorGraph factors) {
        Set<Double> timestamps = new TreeSet<>();
        for (Corners corner : corners) {
            projection(corner, factors);
            timestamps.add(corner.timestamp());
        }
        return timestamps;
    }

    /** Add a planar projection factor for this measurement */
    private static void projection(
            Corners corner,
            NonlinearFactorGraph m_newFactors) {
        double timestamp = corner.timestamp();
        Key x = Keys.poseKey(timestamp);
        Translation3d landmark = corner.landmark();
        Translation2d measurement = corner.measurement();
        Transform3d offset = corner.offset().plus(CAM);
        Intrinsic intrinsic = corner.intrinsic();
        Distortion distortion = corner.distortion();
        m_newFactors.add(PlanarProjectionFactor1.newPlanarProjectionFactor1(
                x,
                Geometry.toPoint3(landmark),
                Geometry.toPoint2(measurement),
                Geometry.toPose3(offset),
                Geometry.toCal3DS2(intrinsic, distortion),
                PX_NOISE));
    }

}
