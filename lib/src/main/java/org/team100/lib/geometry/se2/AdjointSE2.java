package org.team100.lib.geometry.se2;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class AdjointSE2 {
    /**
     * Returns the 3x3 matrix:
     * 
     * | RR -y |
     * | RR x |
     * | 00 1 |
     * 
     * Our twist vector representation is translation-first,
     * so the zero block above is in the lower-left, not the
     * upper-right, as it would be for a more-common rotation-first
     * twist vector.
     * 
     * Applying this matrix to a twist in the pose frame
     * returns the twist in the parent frame of the pose.
     * 
     * Maybe this should be a transform, not a pose?
     */
    public static Matrix<N3, N3> ad(Pose2d p) {
        Matrix<N2, N2> R = p.getRotation().toMatrix();
        Translation2d t = p.getTranslation();
        double x = t.getX();
        double y = t.getY();
        Matrix<N3, N3> ad = new Matrix<>(Nat.N3(), Nat.N3());
        ad.assignBlock(0, 0, R);
        ad.set(0, 2, y);
        ad.set(1, 2, -x);
        ad.set(2, 2, 1);
        return ad;
    }

    /**
     * The adjoint of the inverse of the pose.
     * 
     * Applying this matrix to a twist in the parent frame
     * of the pose returns the twist in the pose frame.
     */
    public static Matrix<N3, N3> adInv(Pose2d p) {
        return ad(GeometryUtil.inverse(p));
    }
}
