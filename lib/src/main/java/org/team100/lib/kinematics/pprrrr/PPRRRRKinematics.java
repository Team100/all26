package org.team100.lib.kinematics.pprrrr;

import java.util.List;
import java.util.Map;

import org.team100.lib.geometry.pprrrr.PPRRRRConfig;
import org.team100.lib.kinematics.urdf.URDFJoint;
import org.team100.lib.kinematics.urdf.URDFJoint.JointType;
import org.team100.lib.kinematics.urdf.URDFJoint.Limit;
import org.team100.lib.kinematics.urdf.URDFLink;
import org.team100.lib.kinematics.urdf.URDFRobot;
import org.team100.lib.kinematics.urdf.URDFRobot.Solver;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N6;
import org.wpilib.math.util.Nat;

/**
 * 6 DOF mechanism: 3 drivetrain DOF (swerve), 3 arm DOF (like RRR).
 */
public class PPRRRRKinematics {
    private final URDFRobot<N6> m_arm;

    public PPRRRRKinematics(double l1, double l2, double l3) {
        URDFLink floor = new URDFLink("floor");
        URDFLink swerve_x = new URDFLink("swerve_x");
        URDFLink swerve_y = new URDFLink("swerve_y");
        URDFLink swerve_rot = new URDFLink("swerve_rot");
        URDFLink upper_arm_link = new URDFLink("upper_arm_link");
        URDFLink lower_arm_link = new URDFLink("lower_arm_link");
        URDFLink end_effector = new URDFLink("end_effector");
        URDFLink tool_center_point = new URDFLink("tool_center_point");
        URDFJoint swerve_x_joint = new URDFJoint("swerve_x_joint",
                JointType.prismatic, new Limit(1000.0, 0, 16, 0.5),
                floor, swerve_x,
                new Pose3d(), VecBuilder.fill(1, 0, 0));
        URDFJoint swerve_y_joint = new URDFJoint("swerve_y_joint",
                JointType.prismatic, new Limit(1000.0, 0, 8, 0.5),
                swerve_x, swerve_y,
                new Pose3d(), VecBuilder.fill(0, 1, 0));
        URDFJoint swerve_rot_joint = new URDFJoint("swerve_rot_joint",
                JointType.continuous, new Limit(1000.0, 0, 16, 0.5),
                swerve_y, swerve_rot,
                new Pose3d(), VecBuilder.fill(0, 0, 1));
        URDFJoint shoulder = new URDFJoint("shoulder",
                JointType.revolute, new Limit(1000.0, -Math.PI, 0, 0.5),
                swerve_rot, upper_arm_link,
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
                JointType.revolute, new Limit(1000.0, -Math.PI / 2, Math.PI / 2, 0.5),
                lower_arm_link, end_effector,
                new Pose3d(l2, 0, 0, new Rotation3d()), VecBuilder.fill(0, 1, 0));
        // center point is l3 away from the wrist, zero +x
        URDFJoint center_point = new URDFJoint("center_point",
                JointType.fixed, null,
                end_effector, tool_center_point,
                new Pose3d(l3, 0, 0, new Rotation3d()), null);

        m_arm = new URDFRobot<>(
                Solver.CD,
                Nat.N6(),
                "PPRRRR",
                List.of(
                        floor,
                        swerve_x,
                        swerve_y,
                        swerve_rot,
                        upper_arm_link,
                        lower_arm_link,
                        end_effector,
                        tool_center_point),
                List.of(
                        swerve_x_joint,
                        swerve_y_joint,
                        swerve_rot_joint,
                        shoulder,
                        elbow,
                        wrist,
                        center_point),
                VecBuilder.fill(0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
    }

    public Pose3d forward(PPRRRRConfig q) {
        Map<String, Double> qMap = Map.of(
                "swerve_x_joint", q.q1(),
                "swerve_y_joint", q.q2(),
                "swerve_rot_joint", q.q3(),
                "shoulder", q.q4(),
                "elbow", q.q5(),
                "wrist", q.q6());
        Map<String, Pose3d> poses = m_arm.forward(qMap);
        return poses.get("center_point");
    }

    public PPRRRRConfig inverse(Pose3d x) {
        // will this work without the initial value?
        Vector<N6> q0 = VecBuilder.fill(0, 0, 0, 0, 0, 0);
        Map<String, Double> qMap = m_arm.inverse(q0, "center_point", x);
        return new PPRRRRConfig(
                qMap.get("swerve_x_joint"),
                qMap.get("swerve_y_joint"),
                qMap.get("swerve_rot_joint"),
                qMap.get("shoulder"),
                qMap.get("elbow"),
                qMap.get("wrist"));
    }
}
