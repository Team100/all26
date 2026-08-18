package org.team100.lib.trajectory.se3;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.se3.WaypointSE3;
import org.team100.lib.path.se3.PathSE3Point;
import org.team100.lib.trajectory.TrajectoryUtil;
import org.team100.lib.trajectory.se3.constraint.TimingConstraintSE3;

import edu.wpi.first.math.geometry.Pose3d;

public class TrajectorySE3 {
    private final List<TrajectorySE3Entry> m_points;
    public final List<TimingConstraintSE3> m_constraints;
    private final double m_duration;

    public TrajectorySE3() {
        m_points = new ArrayList<>();
        m_constraints = new ArrayList<>();
        m_duration = 0;
    }

    public TrajectorySE3(
            List<TrajectorySE3Entry> points, List<TimingConstraintSE3> constraints) {
        m_points = points;
        m_constraints = constraints;
        m_duration = m_points.get(m_points.size() - 1).point().time();
    }

    public TrajectorySE3Entry sample(double timeS) {
        // This scans the whole trajectory for every sample, but most of the time
        // is the interpolation; I tried a TreeMap index and it only saved a few
        // nanoseconds per call.
        if (isEmpty())
            throw new IllegalStateException("can't sample an empty trajectory");
        if (timeS >= m_duration) {
            return getLastPoint();
        }
        if (timeS <= 0) {
            return getPoint(0);
        }

        for (int i0 = 0; i0 < length() - 1; ++i0) {
            int i1 = i0 + 1;
            final TrajectorySE3Entry ceil = getPoint(i1);
            if (ceil.point().time() >= timeS) {
                final TrajectorySE3Entry floor = getPoint(i0);
                double span = ceil.point().time() - floor.point().time();
                if (Math.abs(span) <= 1e-12) {
                    return ceil;
                }
                double delta_t = timeS - floor.point().time();
                return TrajectoryUtil.interpolate(floor, ceil, delta_t);
            }
        }
        throw new IllegalStateException("impossible trajectory: " + toString());
    }

    public int length() {
        return m_points.size();
    }

    public boolean isEmpty() {
        return m_points.isEmpty();
    }

    public TrajectorySE3Entry getPoint(int index) {
        return m_points.get(index);
    }

    public TrajectorySE3Entry getLastPoint() {
        return m_points.get(length() - 1);
    }

    public double duration() {
        return m_duration;
    }

    public void dump() {
        System.out.println("i, s, t, v, a, k, x, y");
        for (int i = 0; i < length(); ++i) {
            TrajectorySE3Entry ts = getPoint(i);
            PathSE3Point pwm = ts.point().point();
            WaypointSE3 w = pwm.waypoint();
            Pose3d p = w.pose();
            System.out.printf("%d, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
                    i, ts.parameter().s(), ts.point().time(), ts.point().velocity(), ts.point().accel(),
                    p.getX(), p.getY());
        }
    }

}
