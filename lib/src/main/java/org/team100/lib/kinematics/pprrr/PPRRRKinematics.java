package org.team100.lib.kinematics.pprrr;

import java.util.List;
import java.util.Map;

import org.team100.lib.geometry.pprrr.PPRRRConfig;
import org.team100.lib.kinematics.urdf.URDFJoint;
import org.team100.lib.kinematics.urdf.URDFJoint.JointType;
import org.team100.lib.kinematics.urdf.URDFJoint.Limit;
import org.team100.lib.kinematics.urdf.URDFLink;
import org.team100.lib.kinematics.urdf.URDFRobot;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N5;
import org.wpilib.math.util.Nat;

/**
 * 5 DOF mechanism: 3 drivetrain DOF (swerve), 2 arm DOF (like RR).
 * 
 * There are only 2 arm DOF to avoid redundancy, which is a bit
 * harder to control.
 */
public class PPRRRKinematics {
    private final URDFRobot<N5> m_arm;

    /**
     * @param l1 length of proximal link
     * @param l2 length of distal link
     */
    public PPRRRKinematics(double l1, double l2) {
        URDFLink floor = new URDFLink("floor");
        URDFLink swerve_x = new URDFLink("swerve_x");
        URDFLink swerve_y = new URDFLink("swerve_y");
        URDFLink swerve_rot = new URDFLink("swerve_rot");
        URDFLink upper_arm_link = new URDFLink("upper_arm_link");
        URDFLink lower_arm_link = new URDFLink("lower_arm_link");
        URDFLink tool_center_point = new URDFLink("tool_center_point");
        URDFJoint swerve_x_joint = new URDFJoint("swerve_x_joint",
                JointType.prismatic, new Limit(1000.0, 0, 16, 0.5),
                floor, swerve_x,
                new Pose3d(), VecBuilder.fill(1, 0, 0));
        URDFJoint swerve_y_joint = new URDFJoint("swerve_y_joint",
                JointType.prismatic, new Limit(1000.0, 0, 8, 0.5),
                swerve_x, swerve_y,
                new Pose3d(), VecBuilder.fill(0, 1, 0));
        // continuous joint is not limited; the limits here
        // are for the solver restart.
        URDFJoint swerve_rot_joint = new URDFJoint("swerve_rot_joint",
                JointType.continuous, new Limit(1000.0, -Math.PI, Math.PI, 0.5),
                swerve_y, swerve_rot,
                new Pose3d(), VecBuilder.fill(0, 0, 1));
        URDFJoint shoulder = new URDFJoint("shoulder",
                JointType.revolute, new Limit(1000.0, -3, -0.1, 0.5),
                swerve_rot, upper_arm_link,
                new Pose3d(), VecBuilder.fill(0, 1, 0));
        // elbow is l1 away from shoulder, angle zero is +x axis
        // range is always "down", like an excavator.
        URDFJoint elbow = new URDFJoint("elbow",
                JointType.revolute, new Limit(1000.0, 0.1, 3, 0.5),
                upper_arm_link, lower_arm_link,
                new Pose3d(l1, 0, 0, new Rotation3d()), VecBuilder.fill(0, 1, 0));
        // center point is l3 away from the wrist, zero +x
        URDFJoint center_point = new URDFJoint("center_point",
                JointType.fixed, null,
                lower_arm_link, tool_center_point,
                new Pose3d(l2, 0, 0, new Rotation3d()), null);

        // Coordinate-descent seems to work better than Newton.
        m_arm = new URDFRobot<>(
                URDFRobot.Solver.CD,
                Nat.N5(),
                "PPRRR",
                List.of(
                        floor,
                        swerve_x,
                        swerve_y,
                        swerve_rot,
                        upper_arm_link,
                        lower_arm_link,
                        tool_center_point),
                List.of(
                        swerve_x_joint,
                        swerve_y_joint,
                        swerve_rot_joint,
                        shoulder,
                        elbow,
                        center_point),
                VecBuilder.fill(1, 1, 0.1, 0.1, 0.1));
    }

    public Pose3d forward(PPRRRConfig q) {
        Map<String, Double> qMap = Map.of(
                "swerve_x_joint", q.q1(),
                "swerve_y_joint", q.q2(),
                "swerve_rot_joint", q.q3(),
                "shoulder", q.q4(),
                "elbow", q.q5());
        Map<String, Pose3d> poses = m_arm.forward(qMap);
        return poses.get("center_point");
    }

    /** Initial value is with arm half-up */
    public PPRRRConfig inverse(Pose3d x) {
        Vector<N5> q0 = VecBuilder.fill(0, 0, 0, -1.5, 1.5);
        Map<String, Double> qMap = m_arm.inverse(
                q0, "center_point", x);
        return new PPRRRConfig(
                qMap.get("swerve_x_joint"),
                qMap.get("swerve_y_joint"),
                qMap.get("swerve_rot_joint"),
                qMap.get("shoulder"),
                qMap.get("elbow"));
    }

    public PPRRRConfig inverse(Pose3d x, Vector<N5> q0) {
        Map<String, Double> qMap = m_arm.inverse(
                q0, "center_point", x);
        return new PPRRRConfig(
                qMap.get("swerve_x_joint"),
                qMap.get("swerve_y_joint"),
                qMap.get("swerve_rot_joint"),
                qMap.get("shoulder"),
                qMap.get("elbow"));
    }
}
