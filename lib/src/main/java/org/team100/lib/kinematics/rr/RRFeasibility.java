package org.team100.lib.kinematics.rr;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRPosition;

/** Selects feasible configurations. */
public class RRFeasibility {
    private static final boolean DEBUG = false;

    private final RRKinematics m_k;

    public RRFeasibility(RRKinematics k) {
        m_k = k;
    }

    public List<RRConfig> filter(List<RRConfig> ql) {
        List<RRConfig> result = new ArrayList<>();
        for (RRConfig q : ql) {
            if (!qRange(q)) {
                continue;
            }
            if (!xRange(q)) {
                continue;
            }
            result.add(q);
        }
        return result;
    }

    /**
     * True if the joints configurations are in their allowed ranges.
     * 
     * TODO: make this match the real mechanism.
     */
    boolean qRange(RRConfig q) {
        if (q.q1() < -Math.PI / 2 || q.q1() > Math.PI / 2) {
            if (DEBUG)
                System.out.printf("q1 out of range %s\n", q.q1());
            return false;
        }
        if (q.q2() < -3 || q.q2() > 3) {
            if (DEBUG)
                System.out.printf("q2 out of range %s\n", q.q2());
            return false;
        }
        return true;
    }

    /**
     * True if the joint workspace positions are ok.
     * 
     * TODO: make a real work envelope
     */
    boolean xRange(RRConfig q) {
        RRPosition x = m_k.forward(q);
        if (x.p2().getX() < 0) {
            if (DEBUG)
                System.out.printf("p2 out of range %s\n", x.p2());
            return false;
        }
        return true;
    }
}
