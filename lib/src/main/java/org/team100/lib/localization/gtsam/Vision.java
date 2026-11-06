package org.team100.lib.localization.gtsam;

import java.util.List;
import java.util.Map.Entry;
import java.util.function.Supplier;

import org.team100.lib.gtsam.util.Geometry;
import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.util.Math100;
import org.team100.lib.util.TimeInterpolatableBuffer100;

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
import gtsam.shared_ptr;

/**
 * TODO: remove the odometry stuff from this class, do it elsewhere.
 */
public class Vision {

    /**
     * TODO: move the constants here to the class, make this not static.
     */
    static void vision(
            SwerveDriveKinematics100 m_kinematics,
            shared_ptr<PlanarGyroParams> m_params,
            TimeInterpolatableBuffer100<OdoGyroMeasurement> m_measurements,
            Values m_result,
            Supplier<List<Corners>> m_vision,
            NonlinearFactorGraph m_newFactors,
            Values m_newValues,
            KeyTimestampMap m_newTimestamps) {
        // look through the pending vision input
        // note there can be multiple records here with the same timestamp.
        for (Corners v : m_vision.get()) {
            double timestamp = v.timestamp();
            Entry<Double, OdoGyroMeasurement> before = m_measurements.lowerEntry(timestamp);
            if (before == null) {
                continue;
            }
            Entry<Double, OdoGyroMeasurement> after = m_measurements.higherEntry(timestamp);
            if (after == null) {
                continue;
            }

            Point3 landmark = Geometry.toPoint3(v.landmark());
            Point2 measurement = Geometry.toPoint2(v.measurement());
            Pose3 offset = Geometry.toPose3(v.offset().plus(GtsamStateEstimator.CAM));
            Cal3DS2 calib = Geometry.toCal3DS2(v.intrinsic(), v.distortion());

            Key x = GtsamStateEstimator.poseKey(timestamp);
            Key b = GtsamStateEstimator.biasKey(timestamp);

            m_newFactors.add(PlanarProjectionFactor1.newPlanarProjectionFactor1(
                    x,
                    landmark,
                    measurement,
                    offset,
                    calib,
                    GtsamStateEstimator.m_visionNoise));

            if (m_newValues.exists(x)) {
                // We've already made a new value for this timestamp.
                continue;
            }
            double beforeTime = before.getKey();
            Key x0 = GtsamStateEstimator.poseKey(beforeTime);
            if (x.j == x0.j) {
                // the vision timestamp happens to be the same as the floor
                continue;
            }
            double afterTime = after.getKey();
            Key x1 = GtsamStateEstimator.poseKey(afterTime);
            if (x.j == x1.j) {
                // the vision timestamp happens to be the same as the ceiling
                continue;
            }

            // make a new value in between
            // and wire up interpolated odometry factors

            Key b0 = GtsamStateEstimator.biasKey(beforeTime);
            Key b1 = GtsamStateEstimator.biasKey(afterTime);
            double dt0 = timestamp - beforeTime;
            double dt1 = afterTime - timestamp;
            // Interpolated measurement for this timestamp
            OdoGyroMeasurement m = m_measurements.get(timestamp);
            OdoGyroMeasurement m0 = before.getValue();
            OdoGyroMeasurement m1 = after.getValue();
            Rot2 dr0 = new Rot2(m.gyro().minus(m0.gyro()).getRadians());
            Rot2 dr1 = new Rot2(m1.gyro().minus(m.gyro()).getRadians());
            m_newFactors.add(Odometry.getOdometryFactor(m_kinematics,
                    x0, x, m0.odo(), m.odo()));
            m_newFactors.add(Odometry.getOdometryFactor(m_kinematics,
                    x, x1, m.odo(), m1.odo()));
            m_newFactors.add(PlanarGyroFactor.FromRotation(
                    x0, x, b0, m_params, dr0, dt0));
            m_newFactors.add(PlanarGyroFactor.FromRotation(
                    x, x1, b, m_params, dr1, dt1));

            m_newFactors.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(
                    b0, b, m_params));
            m_newFactors.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(
                    b, b1, m_params));

            // for interpolation
            double s = dt0 / (afterTime - beforeTime);

            // interpolate the estimated pose to come up with an initial value.
            Pose2 beforePoseEstimate = m_result.atPose2(x0);
            Pose2 afterPoseEstimate = m_result.atPose2(x1);
            Pose2 poseEstimate = Geometry.interpolate(
                    beforePoseEstimate, afterPoseEstimate, s);
            m_newValues.insert(x, poseEstimate);

            // interpolate the estimated bias to come up with an initial value
            double beforeBiasEstimate = m_result.atDouble(b0);
            double afterBiasEstimate = m_result.atDouble(b1);
            double biasEstimate = Math100.interpolate(
                    beforeBiasEstimate, afterBiasEstimate, s);
            m_newValues.insert(b, biasEstimate);

            m_newTimestamps.put(x, timestamp);
            m_newTimestamps.put(b, timestamp);
        }
    }

}
