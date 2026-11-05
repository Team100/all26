package org.team100.lib.geometry;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point3;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

public class OpenCVUtil {

    /**
     * The extrinsic matrix is the inverse of the camera pose.
     */
    public static Transform3d extrinsic(Pose3d cameraPose) {
        return new Transform3d(Pose3d.kZero, cameraPose).inverse();
    }

    /**
     * Rotation vector from extrinsic matrix
     */
    public static Mat getRvec(Transform3d extrinsic) {
        Matrix<N3, N3> r = extrinsic.getRotation().toMatrix();
        Mat rmat = new Mat(3, 3, CvType.CV_64F);
        rmat.put(0, 0, r.get(0, 0));
        rmat.put(0, 1, r.get(0, 1));
        rmat.put(0, 2, r.get(0, 2));
        rmat.put(1, 0, r.get(1, 0));
        rmat.put(1, 1, r.get(1, 1));
        rmat.put(1, 2, r.get(1, 2));
        rmat.put(2, 0, r.get(2, 0));
        rmat.put(2, 1, r.get(2, 1));
        rmat.put(2, 2, r.get(2, 2));
        Mat rvec = new Mat(3, 1, CvType.CV_64F);
        Calib3d.Rodrigues(rmat, rvec);
        return rvec;
    }

    /**
     * Translation from extrinsic matrix
     */
    public static Mat getTVec(Transform3d extrinsic) {
        Translation3d t = extrinsic.getTranslation();
        Mat tVec = Mat.zeros(3, 1, CvType.CV_64F);
        tVec.put(0, 0, t.getX(), t.getY(), t.getZ());
        return tVec;
    }

    public static Point3 point(Translation3d t) {
        return new Point3(t.getX(), t.getY(), t.getZ());
    }

}
