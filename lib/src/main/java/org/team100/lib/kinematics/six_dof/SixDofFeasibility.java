package org.team100.lib.kinematics.six_dof;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;
import org.team100.lib.util.StrUtil;

/** Selects feasible configurations. */
public class SixDofFeasibility {
    private static final boolean DEBUG = false;
    private final SixDofKinematics m_k;

    private final SixDofConfig l = new SixDofConfig(
            -Math.PI, 0, -Math.PI,
            -Math.PI, -Math.PI / 2, -Math.PI);
    private final SixDofConfig u = new SixDofConfig(
            Math.PI, Math.PI, Math.PI,
            Math.PI, Math.PI / 2, Math.PI);

    public SixDofFeasibility(SixDofKinematics k) {
        m_k = k;
    }

    /** Return the list without infeasible configurations. */
    public List<SixDofConfig> filter(List<SixDofConfig> ql) {
        List<SixDofConfig> result = new ArrayList<>();
        for (SixDofConfig q : ql) {
            if (!qRange(q)) {
                if (DEBUG)
                    System.out.println("skipping out-of-joint-range " + q.toString());
                continue;
            }
            if (!xRange(q)) {
                if (DEBUG)
                    System.out.println("skipping out-of-workspace " + q.toString());
                continue;
            }
            result.add(q);
        }
        return result;
    }

    /**
     * True if the joints configurations are in their allowed ranges.
     */
    boolean qRange(SixDofConfig q) {
        if (q.q1() < l.q1() || q.q1() > u.q1()) {
            if (DEBUG)
                System.out.println("swing limit exceeded: " + q.q1());
            return false;
        }
        if (q.q2() < l.q2() || q.q2() > u.q2()) {
            if (DEBUG)
                System.out.println("shoulder limit exceeded: " + q.q2());
            return false;
        }
        if (q.q3() < l.q3() || q.q3() > u.q3()) {
            if (DEBUG)
                System.out.println("elbow limit exceeded: " + q.q3());
            return false;
        }
        if (q.q4() < l.q4() || q.q4() > u.q4()) {
            if (DEBUG)
                System.out.println("wrist roll limit exceeded: " + q.q4());
            return false;
        }
        if (q.q5() < l.q5() || q.q5() > u.q5()) {
            if (DEBUG)
                System.out.println("pitch limit exceeded: " + q.q5());
            return false;
        }
        if (q.q6() < l.q6() || q.q6() > u.q6()) {
            if (DEBUG)
                System.out.println("tool roll limit exceeded: " + q.q6());
            return false;
        }
        return true;
    }

    /**
     * True if the joint workspace positions are ok,
     * 
     * For now this means "above the floor".
     */
    boolean xRange(SixDofConfig q) {
        SixDofPose p = m_k.forward(q);
        // p1 z is fixed
        if (p.p2().getZ() < 0) {
            if (DEBUG)
                System.out.println("elbow pose out of range: " + StrUtil.poseStr(p.p2()));
            return false;
        }
        if (p.p3().getZ() < 0) {
            if (DEBUG)
                System.out.println("wrist origin pose out of range: " + StrUtil.poseStr(p.p3()));
            return false;
        }
        if (p.p4().getZ() < 0) {
            if (DEBUG)
                System.out.println("p4 pose out of range: " + StrUtil.poseStr(p.p4()));
            return false;
        }
        if (p.p5().getZ() < 0) {
            if (DEBUG)
                System.out.println("p5 pose out of range: " + StrUtil.poseStr(p.p5()));
            return false;
        }
        if (p.p6().getZ() < 0) {
            if (DEBUG)
                System.out.println("p6 pose out of range: " + StrUtil.poseStr(p.p6()));
            return false;
        }
        if (p.p7().getZ() < 0) {
            if (DEBUG)
                System.out.println("tcp pose out of range: " + StrUtil.poseStr(p.p7()));
            return false;
        }
        return true;
    }
}
