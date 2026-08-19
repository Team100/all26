package org.team100.lib.trajectory.se2;

import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.XYSeries;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.state.ControlSE2;

import org.wpilib.math.geometry.Rotation2d;

public class TrajectorySE2ToVectorSeries {
    private static final boolean DEBUG = false;

    private final int POINTS;
    /** Length of the vector indicating heading */
    private final double m_scale;

    public TrajectorySE2ToVectorSeries(double scale) {
        this(scale, 50);
    }

    public TrajectorySE2ToVectorSeries(double scale, int points) {
        m_scale = scale;
        POINTS = points;
    }

    /** Maps x to x, y to y */
    public List<VectorSeries> convert(TrajectorySE2 t) {
        VectorSeries s = new VectorSeries("trajectory");
        double duration = t.duration();
        if (DEBUG)
            System.out.printf("duration %f\n", duration);
        double dt = duration / POINTS;
        for (double time = 0; time < duration; time += dt) {
            TrajectorySE2Entry p = t.sample(time);
            WaypointSE2 pp = p.point().point().waypoint();
            double x = pp.pose().getTranslation().getX();
            double y = pp.pose().getTranslation().getY();
            Rotation2d heading = pp.pose().getRotation();
            double dx = m_scale * heading.getCos();
            double dy = m_scale * heading.getSin();
            s.add(x, y, dx, dy);
            if (DEBUG)
                System.out.printf("t %f pp %s\n", time, pp);
        }
        return List.of(s);
    }

    public List<VectorSeries> accel(TrajectorySE2 trajectory) {
        VectorSeries series = new VectorSeries("trajectory");
        double duration = trajectory.duration();
        double dt = duration / POINTS;
        for (double time = 0; time < duration; time += dt) {
            TrajectorySE2Point point = trajectory.sample(time).point();
            ControlSE2 control = point.control();
            double x = control.x().x();
            double y = control.y().x();
            double ax = m_scale * control.x().a();
            double ay = m_scale * control.y().a();
            series.add(x, y, ax, ay);
        }
        return List.of(series);
    }

    /**
     * X as a function of t.
     * 
     * @return (t, x)
     */
    XYSeries x(String name, TrajectorySE2 trajectory) {
        XYSeries series = new XYSeries(name);
        double duration = trajectory.duration();
        double dt = duration / POINTS;
        for (double t = 0; t <= duration + 0.0001; t += dt) {
            TrajectorySE2Entry p = trajectory.sample(t);
            WaypointSE2 pp = p.point().point().waypoint();
            double x = pp.pose().getTranslation().getX();
            series.add(t, x);
        }
        return series;
    }

    /**
     * X dot: dx/dt, as a function of t.
     * 
     * @return (t, \dot{x})
     */
    public XYSeries xdot(String name, TrajectorySE2 trajectory) {
        XYSeries series = new XYSeries(name);
        double duration = trajectory.duration();
        double dt = duration / POINTS;
        for (double t = 0; t <= duration + 0.0001; t += dt) {
            TrajectorySE2Entry p = trajectory.sample(t);
            TrajectorySE2Point pp = p.point();
            WaypointSE2 waypoint = pp.point().waypoint();
            Rotation2d course = waypoint.course().toRotation();
            double velocityM_s = pp.velocity();
            double xv = course.getCos() * velocityM_s;
            if (DEBUG)
                System.out.printf("x %f pp %s\n", t, pp);
            series.add(t, xv);
        }
        return series;
    }

    /**
     * X dot: dx/dt, as a function of x.
     * 
     * @return (x, \dot{x})
     */
    public XYSeries xdotVx(String name, TrajectorySE2 trajectory) {
        XYSeries series = new XYSeries(name);
        double duration = trajectory.duration();
        double dt = duration / POINTS;
        for (double t = 0; t <= duration + 0.0001; t += dt) {
            TrajectorySE2Entry p = trajectory.sample(t);
            TrajectorySE2Point pp = p.point();
            Rotation2d course = pp.point().waypoint().course().toRotation();
            double velocityM_s = pp.velocity();
            WaypointSE2 waypoint = pp.point().waypoint();
            double x = waypoint.pose().getTranslation().getX();
            double xv = course.getCos() * velocityM_s;
            if (DEBUG)
                System.out.printf("x %f pp %s\n", x, pp);
            series.add(x, xv);
        }
        return series;
    }

}
