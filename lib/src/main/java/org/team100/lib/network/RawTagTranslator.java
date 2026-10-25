package org.team100.lib.network;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.camera.Camera;
import org.team100.lib.localization.Blip;
import org.team100.lib.network.RawTags.TagMeasurement;

/** Transform blips into transforms, for testing. */
public class RawTagTranslator {
    List<TagMeasurement> convert(Camera camera, Blip[] value) {
        List<TagMeasurement> measurements = new ArrayList<>();
        for (Blip b : value) {
            double timeSec = (double) b.getTimestamp() / 1e6;
            measurements.add(new TagMeasurement(timeSec, b.blipToTransform()));
        }
        return measurements;
    }
}
