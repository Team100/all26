package org.team100.lib.targeting;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.camera.Camera;
import org.team100.lib.camera.Offset;
import org.team100.lib.state.StateSE2;
import org.team100.lib.targeting.Targets.TargetMeasurement;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Transform sightings into field-relative targets.
 */
public class TargetTranslator {
    private static final boolean DEBUG = false;

    /** Ignore sightings farther away than this. */
    private static final double MAX_DISTANCE = 4.0;

    /** state = f(takt seconds) from history. */
    private final DoubleFunction<StateSE2> m_history;

    public TargetTranslator(DoubleFunction<StateSE2> history) {
        m_history = history;
    }

    List<TargetMeasurement> convert(Camera camera, Target[] sights) {
        List<TargetMeasurement> measurements = new ArrayList<>();
        for (Target sight : sights) {
            // server timestamp in sec
            double timeSec = (double) sight.getTimestamp() / 1e6;

            Pose2d robotPose = m_history.apply(timeSec).pose();
            Transform3d cameraOffset = Offset.get(camera).offset();
            Optional<Translation2d> ot = TargetLocalizer.cameraRotToFieldRelative(
                    robotPose,
                    cameraOffset,
                    sight.sight());
            if (ot.isEmpty())
                continue;
            Translation2d t = ot.get();
            double distance = t.getDistance(robotPose.getTranslation());
            if (distance > MAX_DISTANCE) {
                if (DEBUG)
                    System.out.println("Target is too far away.");
                continue;
            }
            measurements.add(new TargetMeasurement(timeSec, t));
        }
        return measurements;
    }

}
