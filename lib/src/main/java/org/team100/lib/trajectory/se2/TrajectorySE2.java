package org.team100.lib.trajectory.se2;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.trajectory.TrajectoryUtil;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A trajectory in SE(2), the space Pose2d lives in.
 * 
 * A trajectory is a path and a schedule, represented here as a list of
 * TimedState.
 */
public class TrajectorySE2 {
    private final List<TrajectorySE2Entry> m_points;
    private final double m_duration;

    public TrajectorySE2() {
        m_points = new ArrayList<>();
        m_duration = 0;
    }

    /** First timestamp must be zero. */
    public TrajectorySE2(List<TrajectorySE2Entry> points) {
        m_points = points;
        m_duration = m_points.get(m_points.size() - 1).point().time();
    }

    /**
     * Interpolate a TimedState.
     * 
     * @param timeS start is zero.
     */
    public TrajectorySE2Entry sample(double timeS) {
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
            final TrajectorySE2Entry ceil = getPoint(i1);
            if (ceil.point().time() >= timeS) {
                final TrajectorySE2Entry floor = getPoint(i0);
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

    /** Time is at or beyond the trajectory duration. */
    public boolean isDone(double timeS) {
        return timeS >= duration();
    }

    public boolean isEmpty() {
        return m_points.isEmpty();
    }

    public int length() {
        return m_points.size();
    }

    public TrajectorySE2Entry getLastPoint() {
        return m_points.get(length() - 1);
    }

    public List<TrajectorySE2Entry> getPoints() {
        return m_points;
    }

    public TrajectorySE2Entry getPoint(int index) {
        return m_points.get(index);
    }

    public double duration() {
        return m_duration;
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < length(); ++i) {
            builder.append(i);
            builder.append(": ");
            builder.append(getPoint(i));
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }

    /** For cutting-and-pasting into a spreadsheet */
    public void dump() {
        System.out.println("i, s, t, v, a, k, x, y");
        for (int i = 0; i < length(); ++i) {
            TrajectorySE2Entry ts = getPoint(i);
            TrajectorySE2Point point = ts.point();
            PathSE2Point pwm = point.point();
            WaypointSE2 w = pwm.waypoint();
            Pose2d p = w.pose();
            System.out.printf("%d, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
                    i, point.time(), point.velocity(), point.accel(), pwm.K().norm(),
                    p.getX(), p.getY());
        }
    }

    /** For cutting-and-pasting into a spreadsheet */
    public void tdump() {
        System.out.println("t, v, a, k, x, y");
        for (double t = 0; t < duration(); t += 0.02) {
            TrajectorySE2Entry ts = sample(t);
            PathSE2Point pwm = ts.point().point();
            WaypointSE2 w = pwm.waypoint();
            Pose2d p = w.pose();
            System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
                    ts.point().time(), ts.point().velocity(), ts.point().accel(), pwm.K().norm(), p.getX(),
                    p.getY());
        }
    }
}
