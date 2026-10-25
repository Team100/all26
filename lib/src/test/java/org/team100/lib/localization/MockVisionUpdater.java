package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.uncertainty.NoisyPose2d;

import edu.wpi.first.math.geometry.Pose2d;

/** For testing */
public class MockVisionUpdater implements VisionEstimator {
    public final List<Pose2d> poseEstimate = new ArrayList<Pose2d>();
    public final List<Double> timeEstimate = new ArrayList<Double>();

    @Override
    public SwerveState estimate(double t, NoisyPose2d p) {
        poseEstimate.add(p.pose());
        timeEstimate.add(t);
        return null;
    }

    public int size() {
        return poseEstimate.size();
    }
}
