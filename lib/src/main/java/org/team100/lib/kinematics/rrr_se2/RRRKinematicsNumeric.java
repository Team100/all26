package org.team100.lib.kinematics.rrr_se2;

import java.util.List;
import java.util.Map;

import org.team100.lib.kinematics.urdf.URDFJoint;
import org.team100.lib.kinematics.urdf.URDFJoint.JointType;
import org.team100.lib.kinematics.urdf.URDFJoint.Limit;
import org.team100.lib.kinematics.urdf.URDFLink;
import org.team100.lib.kinematics.urdf.URDFRobot;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Nat;

/**
 * RRR planar linkage.
 * 
 * Uses URDF numeric inverse.
 * 
 * NOTE! Coordinates are different from other planar examples.
 * 
 * The arm is in the XZ plane.
 * The rotational zero is along +x.
 * Joint rotation is around Y, and thus rotations "up" are negative.
 */
public class RRRKinematicsNumeric {

    private final URDFRobot<N3> m_arm;

    /**
     * @param l1 upper arm length
     * @param l2 lower arm length
     * @param l3 "hand" length
     */
    public RRRKinematicsNumeric(double l1, double l2, double l3) {
        URDFLink base = new URDFLink("base");
        URDFLink upper_arm_link = new URDFLink("upper_arm_link");
        URDFLink lower_arm_link = new URDFLink("lower_arm_link");
        URDFLink end_effector = new URDFLink("end_effector");
        URDFLink tool_center_point = new URDFLink("tool_center_point");
        // Shoulder origin is zero, axis is along y.
        // range is from all-forward to all-back
        URDFJoint shoulder = new URDFJoint("shoulder",
                JointType.revolute, new Limit(1000.0, -Math.PI, 0, 0.5),
                base, upper_arm_link,
                new Pose3d(), VecBuilder.fill(0, 1, 0));
        // elbow is l1 away from shoulder, angle zero is +x axis
        // range is always "down", like an excavator.
        URDFJoint elbow = new URDFJoint("elbow",
                JointType.revolute, new Limit(1000.0, 0, Math.PI, 0.5),
                upper_arm_link, lower_arm_link,
                new Pose3d(l1, 0, 0, new Rotation3d()), VecBuilder.fill(0, 1, 0));
        // wrist is l2 away from elbow, angle zero is +x axis.
        // range is +/- 90
        URDFJoint wrist = new URDFJoint("wrist",
                JointType.revolute,
                new Limit(1000.0, -Math.PI / 2, Math.PI / 2, 0.5),
                lower_arm_link, end_effector,
                new Pose3d(l2, 0, 0, new Rotation3d()), VecBuilder.fill(0, 1, 0));
        // center point is l3 away from the wrist, zero +x
        URDFJoint center_point = new URDFJoint("center_point",
                JointType.fixed, null,
                end_effector, tool_center_point,
                new Pose3d(l3, 0, 0, new Rotation3d()), null);

        m_arm = new URDFRobot<>(
                URDFRobot.Solver.NEWTON,
                Nat.N3(),
                "RRR",
                List.of(
                        base,
                        upper_arm_link,
                        lower_arm_link,
                        end_effector,
                        tool_center_point),
                List.of(
                        shoulder,
                        elbow,
                        wrist,
                        center_point),
                VecBuilder.fill(1, 1, 1));
    }

    public Pose3d forward(Vector<N3> q) {
        Map<String, Double> qMap = Map.of(
                "shoulder", q.get(0),
                "elbow", q.get(1),
                "wrist", q.get(2));
        Map<String, Pose3d> poses = m_arm.forward(qMap);
        for (Map.Entry<String, Pose3d> e : poses.entrySet()) {
            System.out.printf("%s %s\n", e.getKey(), e.getValue());
        }
        return poses.get("center_point");
    }

    /**
     * 
     * @param x
     * @return (shoulder, elbow, wrist)
     */
    public Vector<N3> inverse(Pose3d x) {
        // will this work without the initial value?
        Vector<N3> q0 = VecBuilder.fill(0, 0, 0);
        Map<String, Double> qMap = m_arm.inverse(q0, "center_point", x);
        return VecBuilder.fill(
                qMap.get("shoulder"),
                qMap.get("elbow"),
                qMap.get("wrist"));
    }

}
