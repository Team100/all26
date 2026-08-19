package org.team100.lib.reference.rn;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.spline.rn.SplineRn;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;
import org.team100.lib.util.StrUtil;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.util.Num;

/** Produces references based on an N-dimensional spline. */
public class SplineReferenceRn<N extends Num> implements ReferenceRn {
    private static final boolean DEBUG = false;
    private final SplineRn<N> m_spline;
    private final double m_duration;
    private double m_startTime;

    public SplineReferenceRn(SplineRn<N> spline, double duration) {
        m_spline = spline;
        m_duration = duration;
    }

    @Override
    public void init() {
        m_startTime = Takt.get();
    }

    @Override
    public List<ModelR1> current() {
        Vector<N> sample = sample(progress());
        List<ModelR1> l = new ArrayList<>();
        for (int i = 0; i < sample.getNumRows(); ++i) {
            l.add(new ModelR1(sample.get(i)));
        }
        return l;
    }

    @Override
    public List<ControlR1> next() {
        Vector<N> sample = sample(progress() + TimedRobot100.LOOP_PERIOD_S);
        List<ControlR1> l = new ArrayList<>();
        for (int i = 0; i < sample.getNumRows(); ++i) {
            l.add(new ControlR1(sample.get(i)));
        }
        return l;
    }

    @Override
    public boolean done() {
        return progress() >= m_duration;
    }

    Vector<N> sample(double t) {
        double s = t / m_duration;
        Vector<N> x = m_spline.x(s);
        if (DEBUG)
            System.out.printf("%f: %s\n", s, StrUtil.vecStr(x));
        return x;
    }

    private double progress() {
        return Takt.get() - m_startTime;
    }

}
