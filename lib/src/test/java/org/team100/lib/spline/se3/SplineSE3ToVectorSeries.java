package org.team100.lib.spline.se3;

import java.util.List;

import org.jfree.chart3d.data.xyz.XYZDataset;
import org.jfree.chart3d.data.xyz.XYZSeriesCollection;
import org.team100.lib.util.ChartUtil3d;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * There is no VectorSeries type in jfree for 3d data, so instead this creates
 * an XYZDataset made up of many XYSSeries, each a little arrow.
 */
public class SplineSE3ToVectorSeries {
    private static final double DS = 0.05;

    private final double m_scale;

    public SplineSE3ToVectorSeries(double scale) {
        m_scale = scale;
    }

    public XYZDataset<String> convert(List<SplineSE3> splines) {
        XYZSeriesCollection<String> dataset = new XYZSeriesCollection<>();
        int j = 0;
        for (int i = 0; i < splines.size(); i++) {
            SplineSE3 spline = splines.get(i);
            for (double s = 0; s <= 1.001; s += DS) {
                Pose3d p = spline.pose(s);
                double x = p.getX();
                double y = p.getY();
                double z = p.getZ();
                Rotation3d heading = p.getRotation();
                Translation3d arrow = new Translation3d(m_scale, 0, 0);
                Translation3d a = arrow.rotateBy(heading);
                double dx = a.getX();
                double dy = a.getY();
                double dz = a.getZ();
                dataset.add(ChartUtil3d.arrow(j, x, y, z, dx, dy, dz));
                j++;
            }
        }
        return dataset;
    }

}
