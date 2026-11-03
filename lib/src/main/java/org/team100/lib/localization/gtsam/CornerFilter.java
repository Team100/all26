package org.team100.lib.localization.gtsam;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

public class CornerFilter {

    public CornerFilter() {
    }

    public List<Corners> filter(List<Corners> measurements) {
        List<Corners> filtered = new ArrayList<>();
        for (Corners m : measurements) {
            if (Experiments.INSTANCE.enabled(Experiment.IgnoreVision)) {
                continue;
            }
            filtered.add(m);
        }
        return filtered;
    }
}
