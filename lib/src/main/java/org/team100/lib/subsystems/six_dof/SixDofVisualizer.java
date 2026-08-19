package org.team100.lib.subsystems.six_dof;

import java.util.List;
import java.util.function.Supplier;

import org.team100.lib.geometry.six_dof.SixDofPose;
import org.team100.lib.visualization.Serial3dVisualization;
import org.wpilib.math.geometry.Pose3d;

/** Use the glass "mechanism" display to show the arm position in 3d. */
public class SixDofVisualizer {

    private final Supplier<SixDofPose> m_arm;

    private final Serial3dVisualization m_viz;

    public SixDofVisualizer(Supplier<SixDofPose> arm) {
        m_arm = arm;
        m_viz = new Serial3dVisualization(this::poses);
    }

    public void periodic() {
        m_viz.periodic();
    }

    List<Pose3d> poses() {
        SixDofPose p = m_arm.get();
        List<Pose3d> pList = List.of(
                Pose3d.kZero,
                p.p1(),
                p.p2(),
                p.p3(),
                p.p4(),
                p.p5(),
                p.p6(),
                p.p7());
        return pList;
    }

}
