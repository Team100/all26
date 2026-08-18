package org.team100.lib.kinematics.urdf;

import java.util.List;

import org.team100.lib.kinematics.urdf.URDFJoint.JointType;
import org.team100.lib.kinematics.urdf.URDFJoint.Limit;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N5;

/**
 * An example of using the URDF model to describe the Lynxmotion arm.
 * 
 * The content came from here:
 * https://groups.google.com/g/moveit-users/c/g3vaTDQSRcQ
 * 
 * I flipped the joint axis signs so that none of them are inverted, since I
 * find that easier to remember.
 * 
 * I changed the joint zeros so that the "arm zero" is the same as the
 * rotation3d zero axis, i.e. +x, and I changed the limits to match.
 */
public class URDFAL5D extends URDFRobot<N5> {

    private URDFAL5D(
            String name,
            List<URDFLink> links,
            List<URDFJoint> joints) {
        super(URDFRobot.Solver.NEWTON, Nat.N5(), name,
                links, joints, VecBuilder.fill(1, 1, 1, 1, 1));
    }

    public static URDFAL5D make() {
        URDFLink base_link = new URDFLink("base_link");
        URDFLink base_pan_link = new URDFLink("base_pan_link");
        URDFLink upper_arm_link = new URDFLink("upper_arm_link");
        URDFLink lower_arm_link = new URDFLink("lower_arm_link");
        URDFLink link3 = new URDFLink("link3");
        URDFLink gripper = new URDFLink("gripper");
        URDFLink tool_center_point = new URDFLink("tool_center_point");
        return new URDFAL5D(
                "AL5D",
                List.of(
                        base_link,
                        base_pan_link,
                        upper_arm_link,
                        lower_arm_link,
                        link3,
                        gripper,
                        tool_center_point),
                List.of(
                        new URDFJoint(
                                "base_pan",
                                JointType.revolute,
                                new Limit(1000, -Math.PI / 2, Math.PI / 2, 0.5),
                                base_link,
                                base_pan_link,
                                // rot z zero used to be -pi/2
                                new Pose3d(0, 0, 0.06731, new Rotation3d(0, 0, 0)),
                                VecBuilder.fill(0, 0, 1)),
                        new URDFJoint(
                                "shoulder_tilt",
                                JointType.revolute,
                                new Limit(1000.0, -Math.PI, 0, 0.5),
                                base_pan_link,
                                upper_arm_link,
                                new Pose3d(),
                                // rotation was -y
                                VecBuilder.fill(0, 1, 0)),
                        new URDFJoint(
                                "elbow_tilt",
                                JointType.revolute,
                                new Limit(1000.0, 0.0, Math.PI, 0.5),
                                upper_arm_link,
                                lower_arm_link,
                                // rot y zero used to be pi
                                new Pose3d(0.14605, 0, 0, new Rotation3d(0, 0, 0)),
                                // rot was -y
                                VecBuilder.fill(0, 1, 0)),
                        new URDFJoint(
                                "wrist_tilt",
                                JointType.revolute,
                                new Limit(1000.0, -Math.PI / 2, Math.PI / 2, 0.5),
                                lower_arm_link,
                                link3,
                                // rot y zero used to be pi/2
                                new Pose3d(0.187325, 0, 0, new Rotation3d(0, 0, 0)),
                                // rot was -y
                                VecBuilder.fill(0, 1, 0)),
                        new URDFJoint(
                                "wrist_rotate",
                                JointType.revolute,
                                new Limit(1000.0, -Math.PI / 2, Math.PI / 2, 0.5),
                                link3,
                                gripper,
                                // rot x zero used to be -pi/2
                                // this is longer than the original i think because of the twist axis
                                new Pose3d(0.061, 0, 0, new Rotation3d(0, 0, 0)),
                                VecBuilder.fill(1, 0, 0)),
                        new URDFJoint(
                                "center_point",
                                JointType.fixed,
                                null,
                                gripper,
                                tool_center_point,
                                // 55mm is the distance to the *end* of the end effector, not the target grip
                                // point
                                new Pose3d(0.055, 0, 0, new Rotation3d()),
                                null)));
    }
}