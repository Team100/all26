package org.team100.lib.subsystems.lynxmotion_arm;

import java.util.List;
import java.util.function.Supplier;

import org.team100.lib.geometry.lynx_arm.LynxArmPose;
import org.team100.lib.visualization.Serial3dVisualization;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Use the glass "mechanism" display to show the arm position in 3d.
 */
public class LynxArmVisualizer {

    private final Supplier<LynxArmPose> m_arm;
    private final Serial3dVisualization m_viz;

    public LynxArmVisualizer(Supplier<LynxArmPose> arm) {
        m_arm = arm;
        m_viz = new Serial3dVisualization(this::poses);
    }

    public void periodic() {
        m_viz.periodic();
    }

    List<Pose3d> poses() {
        LynxArmPose p = m_arm.get();
        List<Pose3d> pList = List.of(
                Pose3d.kZero,
                p.p1(),
                p.p2(),
                p.p3(),
                p.p4(),
                p.p5(),
                p.p6());
        return pList;
    }

}
