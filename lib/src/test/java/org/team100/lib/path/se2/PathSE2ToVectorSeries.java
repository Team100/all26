package org.team100.lib.path.se2;

import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.XYSeries;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;

public class PathSE2ToVectorSeries {
    private static final boolean DEBUG = false;
    private static final int POINTS = 50;

    /** Length of the vector indicating heading */
    private final double m_scale;

    public PathSE2ToVectorSeries(double scale) {
        m_scale = scale;
    }

    /** Maps x to x, y to y */
    public List<VectorSeries> convert(PathSE2 path) {
        VectorSeries s = new VectorSeries("path");
        double l = path.distance(path.length() - 1);
        double dl = l / POINTS;
        for (double d = 0; d < l; d += dl) {
            PathSE2Point point = path.sample(d);
            WindingPose2d p = point.waypoint().pose();
            double x = p.getX();
            double y = p.getY();
            double heading = p.getRotation();
            double dx = m_scale * Math.cos(heading);
            double dy = m_scale * Math.sin(heading);
            if (DEBUG)
                System.out.printf("%f %f %f %f %f\n", d, x, y, dx, dy);
            s.add(x, y, dx, dy);
        }
        return List.of(s);
    }

    public List<VectorSeries> curvature(PathSE2 path) {
        VectorSeries series = new VectorSeries("path");
        double l = path.distance(path.length() - 1);
        double dl = l / POINTS;
        for (double d = 0; d < l; d += dl) {
            PathSE2Point point = path.sample(d);
            WindingPose2d p = point.waypoint().pose();
            double x = p.getX();
            double y = p.getY();
            Vector<N2> K = point.K();
            double dx = m_scale * K.get(0);
            double dy = m_scale * K.get(1);
            series.add(x, y, dx, dy);
        }
        return List.of(series);
    }

    public static XYSeries x(String name, List<PathSE2Point> poses) {
        XYSeries series = new XYSeries(name);
        for (int i = 0; i < poses.size(); ++i) {
            PathSE2Point pose = poses.get(i);
            series.add(i, pose.waypoint().pose().getX());
        }
        return series;
    }

}
