package org.team100.lib.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;

/**
 * Replacements for WPILib geometry toString methods.
 */
public class StrUtil {

    public static String pose2Str(Pose2d p) {
        return String.format("%6.3f, %6.3f, %6.3f",
                p.getX(), p.getY(), p.getRotation().getRadians());
    }

    public static String poseStr(Pose2d p) {
        return String.format("%.8e, %.8e, %.8e",
                p.getX(), p.getY(), p.getRotation().getRadians());
    }

    /** Lots of decimal places. */
    public static String poseStr(Pose3d p) {
        return String.format("%.8e, %.8e, %.8e, %.8e, %.8e, %.8e",
                p.getX(), p.getY(), p.getZ(),
                p.getRotation().getX(), p.getRotation().getY(), p.getRotation().getZ());
    }

    /** Prettier. */
    public static String poseStr2(Pose3d p) {
        return String.format("%5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f",
                p.getX(), p.getY(), p.getZ(),
                p.getRotation().getX(), p.getRotation().getY(), p.getRotation().getZ());
    }

    public static String twistStr(Twist2d t) {
        return String.format("[ %8.3f %8.3f %8.3f ]",
                t.dx, t.dy, t.dtheta);
    }

    public static String twistStr(Twist3d t) {
        return String.format("[ %.8f %.8f %.8f %.8f %.8f %.8f ]",
                t.dx, t.dy, t.dz, t.rx, t.ry, t.rz);
    }

    public static String rotStr(Rotation2d r) {
        return String.format("%9.5f",
                r.getRadians());
    }

    public static String rotStr(Rotation3d r) {
        return String.format("%.8e, %.8e, %.8e",
                r.getX(), r.getY(), r.getZ());
    }

    public static String transStr(Translation2d t) {
        return String.format("%12.8f, %12.8f",
                t.getX(), t.getY());
    }

    public static String transStr(Translation3d t) {
        return String.format("%.8e, %.8e, %.8e",
                t.getX(), t.getY(), t.getZ());
    }

    public static String vecStr(Vector<?> m) {
        StringBuilder b = new StringBuilder();
        b.append("[ ");
        for (int i = 0; i < m.getNumRows(); ++i) {
            b.append(String.format("%8.3f ", m.get(i)));
        }
        b.append("]");
        return b.toString();
    }

    public static String matStr(Matrix<?, ?> m) {
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < m.getNumRows(); ++i) {
            if (i == 0)
                b.append("[");
            else
                b.append(" ");
            for (int j = 0; j < m.getNumCols(); ++j) {
                b.append(String.format(" %8.4f", m.get(i, j)));
            }
            if (i < m.getNumRows() - 1)
                b.append(";\n");
            else
                b.append("]");
        }

        return b.toString();
    }

    public static String arrayStr(double[] x) {
        StringBuilder b = new StringBuilder();
        b.append("[");
        for (int i = 0; i < x.length; ++i) {
            b.append(String.format(" %10.6f", x[i]));
        }
        b.append("]");
        return b.toString();
    }

    private StrUtil() {
        //
    }
}
