package org.team100.lib.trajectory.se3;

import org.jfree.chart3d.data.xyz.XYZDataset;
import org.jfree.chart3d.data.xyz.XYZSeriesCollection;
import org.team100.lib.geometry.se3.WaypointSE3;
import org.team100.lib.util.ChartUtil3d;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * There is no VectorSeries type in jfree for 3d data, so instead this creates
 * an XYZDataset made up of many XYSSeries, each a little arrow.
 */
public class TrajectorySE3ToVectorSeries {
    private static final int POINTS = 20;

    private final double m_scale;

    public TrajectorySE3ToVectorSeries(double scale) {
        m_scale = scale;
    }

    public XYZDataset<String> fromTrajectory(TrajectorySE3 t) {
        XYZSeriesCollection<String> dataset = new XYZSeriesCollection<>();
        double duration = t.duration();
        double dt = duration / POINTS;
        for (double time = 0; time < duration; time += dt) {
            TrajectorySE3Entry p = t.sample(time);
            WaypointSE3 pp = p.point().point().waypoint();
            double x = pp.pose().getTranslation().getX();
            double y = pp.pose().getTranslation().getY();
            double z = pp.pose().getTranslation().getZ();
            Rotation3d heading = pp.pose().getRotation();
            Translation3d arrow = new Translation3d(m_scale, 0, 0);
            Translation3d a = arrow.rotateBy(heading);
            double dx = a.getX();
            double dy = a.getY();
            double dz = a.getZ();
            dataset.add(ChartUtil3d.arrow(time, x, y, z, dx, dy, dz));
        }
        return dataset;
    }

    // some fake data for working on the renderer
    public static XYZDataset<String> data() {
        double segment_size = 0.1;
        XYZSeriesCollection<String> dataset = new XYZSeriesCollection<>();
        double t = 0;
        for (int i = 0; i < 120; ++i) {
            double x = Math.cos(t);
            double y = Math.sin(t);
            double z = t;
            double dx = -Math.sin(t) * segment_size;
            double dy = Math.cos(t) * segment_size;
            double dz = segment_size;
            dataset.add(ChartUtil3d.arrow(i, x, y, z, dx, dy, dz));
            t += segment_size;
        }
        return dataset;
    }

}
