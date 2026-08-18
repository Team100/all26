package org.team100.lib.path.se3;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.Metrics;

import edu.wpi.first.math.geometry.Pose3d;

public class PathSE3 {

    private final List<PathSE3Entry> m_points;
    private final double[] m_distances;

    public PathSE3(final List<PathSE3Entry> states) {
        int n = states.size();
        m_points = new ArrayList<>(n);
        m_distances = new double[n];
        if (states.isEmpty()) {
            return;
        }
        m_distances[0] = 0.0;
        m_points.add(states.get(0));
        for (int i0 = 0; i0 < n - 1; ++i0) {
            int i1 = i0 + 1;
            m_points.add(states.get(i1));
            Pose3d p0 = getEntry(i0).point().waypoint().pose();
            Pose3d p1 = getEntry(i1).point().waypoint().pose();
            double dist = Metrics.translationalDistance(p0, p1);
            m_distances[i1] = m_distances[i0] + dist;
        }
    }

    public int length() {
        return m_points.size();
    }

    public PathSE3Entry getEntry(int index) {
        if (m_points.isEmpty())
            return null;
        return m_points.get(index);
    }

    public double distance(int index) {
        if (m_points.isEmpty())
            return 0;
        return m_distances[index];
    }
}
