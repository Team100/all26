package org.team100.lib.localization.gtsam;

import java.util.Collection;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import org.team100.lib.camera.Distortion;
import org.team100.lib.camera.Intrinsic;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.gtsam.util.Geometry;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.util.Nat;

import gtsam.FixedLagSmoother.KeyTimestampMap;
import gtsam.Key;
import gtsam.NonlinearFactorGraph;
import gtsam.PlanarProjectionFactor1;
import gtsam.Values;
import gtsam.Vector2;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;
import gtsam.noiseModel.Robust;
import gtsam.noiseModel.mEstimator.Huber;

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
    static final shared_ptr<Diagonal> PX_NOISE_LOW = Diagonal.Sigmas(
            new Vector2(2, 2));

    // Pixel noise.
    static final shared_ptr<Diagonal> PX_NOISE_HIGH = Diagonal.Sigmas(
            new Vector2(20, 20));

    // Huber loss is quadratic for small values (like least-squares) but
    // linear for large values (to avoid overweighing them).
    // https://en.wikipedia.org/wiki/Huber_loss
    static final shared_ptr<Robust> PX_NOISE = Robust.Create(
            Huber.Create(2),
            Diagonal.Sigmas(new Vector2(2, 2)));

    /**
     * TODO: move the constants here to the class, make this not static.
     * 
     * @return the list of microsecond timestamps that need pose values, timestamps,
     *         prior factors, and between factors (if possible).
     */
    static Collection<Long> vision(
            Values m_result,
            List<Corners> corners,
            NonlinearFactorGraph newFactors,
            Values newValues,
            KeyTimestampMap newTimestamps) {
        // Accumulate unique timestamps since they can be repeated.
        Set<Long> timestamps = new TreeSet<>();
        for (Corners corner : corners) {
            projection(corner, newFactors, newValues, newTimestamps);
            timestamps.add((long) (corner.timestamp() * 1e6));
        }
        return timestamps;
    }

    /** Add a planar projection factor for this measurement */
    private static void projection(
            Corners corner,
            NonlinearFactorGraph newFactors,
            Values newValues,
            KeyTimestampMap newTimestamps) {
        double timestamp = corner.timestamp();
        Key x = Keys.poseKey(timestamp);
        Translation3d landmark = corner.landmark();
        Translation2d measurement = corner.measurement();
        Transform3d offset = corner.offset().plus(CAM);
        Intrinsic intrinsic = corner.intrinsic();
        Distortion distortion = corner.distortion();

        newFactors.add(PlanarProjectionFactor1.newPlanarProjectionFactor1(
                x,
                Geometry.toPoint3(landmark),
                Geometry.toPoint2(measurement),
                Geometry.toPose3(offset),
                Geometry.toCal3DS2(intrinsic, distortion),
                noise()));
    }

    static shared_ptr<? extends gtsam.noiseModel.Base> noise() {
        if (Experiments.INSTANCE.enabled(Experiment.HighVisionNoise))
            return PX_NOISE_HIGH;
        return PX_NOISE_LOW;
    }

}
