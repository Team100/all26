package org.team100.lib.spline.se2;

import java.util.ArrayList;
import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.XYSeries;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;

public class SplineSE2ToVectorSeries {
    private static final double POINTS = 50;
    /** Length of the vector indicating heading */
    private final double m_scale;

    public SplineSE2ToVectorSeries(double scale) {
        m_scale = scale;
    }

    /**
     * Show little arrows indicating the heading, using uniform steps in the
     * parameter, s.
     * 
     * @return (x, y, dx, dy)
     */
    public List<VectorSeries> convert(List<? extends ISplineSE2> splines) {
        List<VectorSeries> result = new ArrayList<>();
        for (int i = 0; i < splines.size(); i++) {
            ISplineSE2 spline = splines.get(i);
            VectorSeries series = new VectorSeries(String.format("%d", i));
            for (double s = 0; s <= 1.001; s += 1 / POINTS) {
                // Pose2d p = spline.entry(s).point().waypoint().pose();
                Pose2d p = spline.pose(s);
                double x = p.getX();
                double y = p.getY();
                Rotation2d heading = p.getRotation();
                double dx = m_scale * heading.getCos();
                double dy = m_scale * heading.getSin();
                series.add(x, y, dx, dy);
            }
            result.add(series);
        }
        return result;
    }

    /** Show the K vector. */
    public List<VectorSeries> curvature(List<? extends ISplineSE2> splines) {
        List<VectorSeries> result = new ArrayList<>();
        for (int i = 0; i < splines.size(); i++) {
            ISplineSE2 spline = splines.get(i);
            VectorSeries series = new VectorSeries(String.format("%d", i));
            for (double s = 0; s <= 1.001; s += 1 / POINTS) {
                Pose2d p = spline.pose(s);
                double x = p.getX();
                double y = p.getY();
                Vector<N2> K = spline.K(s);
                double dx = m_scale * K.get(0);
                double dy = m_scale * K.get(1);
                series.add(x, y, dx, dy);
            }
            result.add(series);
        }
        return result;
    }

    /**
     * X as a function of s.
     * 
     * @return (s, x)
     */
    public static XYSeries x(String name, List<SplineSE2> splines) {
        XYSeries series = new XYSeries(name);
        for (SplineSE2 spline : splines) {
            for (double s = 0; s <= 1.001; s += POINTS) {
                // double x = spline.entry(s).point().waypoint().pose().getX();
                double x = spline.pose(s).getX();
                series.add(s, x);
            }
        }
        return series;
    }

    /**
     * X prime: dx/ds, as a function of s.
     * 
     * @return (s, x')
     */
    public static XYSeries xPrime(String name, List<SplineSE2> splines) {
        XYSeries series = new XYSeries(name);
        for (SplineSE2 spline : splines) {
            for (double s = 0; s <= 1.001; s += POINTS) {
                double x = spline.dx(s);
                series.add(s, x);
            }
        }
        return series;
    }

    /**
     * X prime prime: d^2x/ds^2, as a function of s.
     * 
     * @return (s, x'')
     */
    public static XYSeries xPrimePrime(String name, List<SplineSE2> splines) {
        XYSeries series = new XYSeries(name);
        for (SplineSE2 spline : splines) {
            for (double s = 0; s <= 1.001; s += POINTS) {
                double x = spline.ddx(s);
                series.add(s, x);
            }
        }
        return series;
    }

}
