package org.team100.lib.kinematics.lynx_arm;

import java.util.Map;

import org.team100.lib.geometry.lynx_arm.LynxArmConfig;
import org.team100.lib.geometry.lynx_arm.LynxArmPose;
import org.team100.lib.kinematics.urdf.URDFAL5D;
import org.team100.lib.kinematics.urdf.URDFRobot;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N5;

/** Uses URDFRobot for the Lynxmotion arm. */
public class NumericLynxArmKinematics implements LynxArmKinematics {

    private final URDFRobot<N5> m_arm;

    public NumericLynxArmKinematics() {
        m_arm = URDFAL5D.make();
    }

    @Override
    public LynxArmPose forward(LynxArmConfig joints) {
        Map<String, Double> qMap = Map.of(
                // positive = pan left, so extent is +x +y
                "base_pan", joints.swing().getAsDouble(),
                "shoulder_tilt", joints.boom(),
                "elbow_tilt", joints.stick(),
                "wrist_tilt", joints.wrist(),
                "wrist_rotate", joints.twist().getAsDouble());
        Map<String, Pose3d> poses = m_arm.forward(qMap);
        LynxArmPose p = new LynxArmPose(
                poses.get("base_pan"),
                poses.get("shoulder_tilt"),
                poses.get("elbow_tilt"),
                poses.get("wrist_tilt"),
                poses.get("wrist_rotate"),
                poses.get("center_point"));
        return p;
    }

    @Override
    public LynxArmConfig inverse(LynxArmConfig initial, Pose3d end) {
        Vector<N5> q0 = initial.toVec();
        Map<String, Double> qMap = m_arm.inverse(q0, "center_point", end);
        LynxArmConfig c = new LynxArmConfig(
                qMap.get("base_pan"),
                qMap.get("shoulder_tilt"),
                qMap.get("elbow_tilt"),
                qMap.get("wrist_tilt"),
                qMap.get("wrist_rotate"));
        return c;
    }

}
