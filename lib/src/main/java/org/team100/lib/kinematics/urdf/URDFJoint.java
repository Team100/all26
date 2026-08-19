package org.team100.lib.kinematics.urdf;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;

/**
 * See https://wiki.ros.org/urdf/XML/joint
 * 
 * The revolute/continuous and prismatic types use a scalar parameter, so they
 * are handled below.
 * 
 * Floating and planar types require a vector parameter, which the solver
 * currently doesn't know how to do.
 * 
 * So if you want floating or planar, use multiple revolute/prismatic joints
 * with zero origin.
 * 
 * TODO: make origin Transform3d not Pose3d.
 * 
 * @param origin Joint origin in the parent link frame. Put another way, this is
 *               the transform representing the parent link.
 * @param axis   Axis of rotation or sliding.
 */
public record URDFJoint(
        String name,
        JointType type,
        Limit limit,
        URDFLink parent,
        URDFLink child,
        Pose3d origin,
        Vector<N3> axis) {
    private static final boolean DEBUG = false;

    /**
     * @param effort
     * @param lower
     * @param upper
     * @param velocity
     */
    public record Limit(double effort, double lower, double upper, double velocity) {
    }

    public enum JointType {
        /** hinge with limits */
        revolute,
        /** hinge without limits */
        continuous,
        /** linear sliding with limits */
        prismatic,
        /** not moving */
        fixed,
        /** free in all 6 dimensions */
        floating,
        /** free in a plane perpendicular to the axis */
        planar
    }

    boolean active() {
        // skip fixed since it doesn't have a parameter and thus breaks the solver
        return type() == JointType.revolute
                || type() == JointType.continuous
                || type() == JointType.prismatic;
    }

    /**
     * Transform for a single joint.
     * The parameter, q, can be null, for fixed joints.
     * First transform is the "origin" transform (in the parent frame), followed by
     * the joint transform (rotation or translation).
     */
    Transform3d transform(Double q) {
        // First, translate along the link, in the parent frame.
        Transform3d linkTransform = new Transform3d(Pose3d.kZero, origin());

        // Then, rotate or translate as appropriate.
        Transform3d jointTransform = switch (type()) {
            case revolute, continuous -> new Transform3d(0, 0, 0, new Rotation3d(axis(), q));
            case prismatic -> new Transform3d(new Translation3d(axis().times(q)), Rotation3d.kZero);
            case fixed -> Transform3d.kZero;
            default -> throw new UnsupportedOperationException();
        };

        if (DEBUG) {
            System.out.printf("linkTransform %s\n",
                    linkTransform);
            System.out.printf("jointTransform %s\n",
                    jointTransform);
        }
        return linkTransform.plus(jointTransform);
    }
}
