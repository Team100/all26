package org.team100.lib.geometry.se3;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;

public class AdjointSE3 {
    /**
     * Returns the 6x6 matrix:
     * 
     * | RRR txR |
     * | RRR txR |
     * | RRR txR |
     * | 000 RRR |
     * | 000 RRR |
     * | 000 RRR |
     * 
     * Our twist vector representation is translation-first,
     * so the zero block above is in the lower-left, not the
     * upper-right, as it would be for the GTSAM rotation-first
     * twist vector.
     * 
     * Applying this matrix to a twist in the pose frame
     * returns the twist in the parent frame of the pose.
     * 
     * Maybe this should be a transform, not a pose?
     */
    public static Matrix<N6, N6> ad(Pose3d p) {
        Matrix<N3, N3> R = p.getRotation().toMatrix();
        Matrix<N3, N3> tx = GeometryUtil.skewSymmetric(p.getTranslation());
        Matrix<N3, N3> txR = tx.times(R);
        Matrix<N6, N6> ad = new Matrix<>(Nat.N6(), Nat.N6());
        ad.assignBlock(0, 0, R);
        ad.assignBlock(0, 3, txR);
        ad.assignBlock(3, 3, R);
        return ad;
    }

    /**
     * The adjoint of the inverse of the pose.
     * 
     * Applying this matrix to a twist in the parent frame
     * of the pose returns the twist in the pose frame.
     */
    public static Matrix<N6, N6> adInv(Pose3d p) {
        return ad(Pose3d.kZero.relativeTo(p));
    }

}
