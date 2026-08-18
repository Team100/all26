package org.team100.lib.kinematics.lynx_arm;

import org.team100.lib.geometry.lynx_arm.LynxArmConfig;
import org.team100.lib.geometry.lynx_arm.LynxArmPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

public interface LynxArmKinematics {
    static final boolean DEBUG = false;

    /**
     * Applies the configuration transforms in order to obtain the end-effector
     * (grip center) pose relative to the arm origin, which is the on the tabletop
     * at the swing axis.
     */
    LynxArmPose forward(LynxArmConfig joints);

    /**
     * Solve the inverse kinematics for the given end-effector pose.
     * 
     * The initial config is useful for the solver: usually the new position is not
     * too far from the old one.
     * 
     * You probably want to "fix" the argument to this function.
     * 
     * Refer to the diagram
     * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
     */
    LynxArmConfig inverse(LynxArmConfig initial, Pose3d end);

    /**
     * The Lynxmotion arm doesn't allow arbitrary end-effector poses. If you supply
     * an "impossible" pose to the inverse kinematics, then the solver will fail.
     * That might be ok -- the "best approach" might be good enough, but it takes a
     * long time for the solver to decide that it's impossible, and it's not
     * necessarily deterministic. Instead, this method constraints the end-effector
     * pose to be feasible.
     * 
     * This method projects the grip axis into the yaw-Z plane, finds the rotation
     * from the grip axis to that rotation, and applies it. This results in less
     * error than the method below.
     */
    static Pose3d fix(Pose3d p) {
        Translation3d x = new Translation3d(1, 0, 0);
        Translation3d rotated = x.rotateBy(p.getRotation());
        // is the resulting vector vertical?
        Translation2d trans2d = rotated.toTranslation2d();
        if (trans2d.getNorm() < 1e-3) {
            // It's vertical, so the yaw can be free
            return p;
        }
        // it's not vertical.
        double yaw = p.getTranslation().toTranslation2d().getAngle().getRadians();
        if (DEBUG) {
            System.out.printf("yaw %s\n", yaw);
        }

        Vector<N3> normal = Vector.cross(
                VecBuilder.fill(Math.cos(yaw), Math.sin(yaw), 0),
                VecBuilder.fill(0, 0, 1));
        Matrix<N3, N3> pR = p.getRotation().toMatrix();
        // where +x vector ends up
        Vector<N3> pRV = new Vector<>(pR.times(VecBuilder.fill(1, 0, 0)));
        // the projection onto the normal vector
        Vector<N3> pRVproj = pRV.projection(normal);
        // the projection in the yaw-Z plane
        Vector<N3> inPlane = pRV.minus(pRVproj);
        // rotation of the original +x vector to the plane projection
        Rotation3d adjustment = new Rotation3d(pRV, inPlane);
        // apply the rotation
        Rotation3d fixedR = p.getRotation().rotateBy(adjustment);
        return new Pose3d(p.getTranslation(), fixedR);
    }

    /**
     * This method adjusts the world-frame yaw to match the yaw of the arm.
     * This produces large errors.
     */
    static Pose3d fixYawOnly(Pose3d p) {
        Translation3d x = new Translation3d(1, 0, 0);
        Translation3d rotated = x.rotateBy(p.getRotation());
        // is the resulting vector vertical?
        Translation2d trans2d = rotated.toTranslation2d();
        if (trans2d.getNorm() < 1e-3) {
            // It's vertical, so the yaw can be free
            return p;
        }
        // it's not vertical.
        double yaw = p.getTranslation().toTranslation2d().getAngle().getRadians();
        if (DEBUG) {
            System.out.printf("yaw %s\n", yaw);
        }
        return new Pose3d(new Translation3d(p.getX(), p.getY(), p.getZ()),
                new Rotation3d(p.getRotation().getX(), p.getRotation().getY(), yaw));

    }

}