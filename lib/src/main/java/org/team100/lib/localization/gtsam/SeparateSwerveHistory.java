package org.team100.lib.localization.gtsam;

import java.util.HashMap;
import java.util.Map;

import org.team100.lib.localization.LocalizerEstimate;
import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.util.KeyBuffer;
import org.team100.lib.util.TimeInterpolatableBuffer100;

/**
 * A version of the history idea that includes some of the GTSAM constraints:
 * discrete, nonqueryable/noninterpolatable estimate store.
 */
public class SeparateSwerveHistory {
    private final double m_bufferDuration;
    private final TimeInterpolatableBuffer100<OdoGyroMeasurement> m_measurements;
    // These are the estimate keys in seconds.
    private final KeyBuffer m_timestamps;
    // Here the estimates are not interpolatable; the key is microseconds.
    private final Map<Long, LocalizerEstimate> m_estimates;

    public SeparateSwerveHistory(
            double bufferDuration,
            double initialTimestamp,
            OdoGyroMeasurement initialMeasurement) {
        m_bufferDuration = bufferDuration;
        m_measurements = new TimeInterpolatableBuffer100<>(
                bufferDuration, initialTimestamp, initialMeasurement);
        m_timestamps = new KeyBuffer();
        m_estimates = new HashMap<>();
    }
}
