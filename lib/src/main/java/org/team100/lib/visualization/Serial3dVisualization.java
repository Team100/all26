package org.team100.lib.visualization;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Nat;
import org.wpilib.math.util.Pair;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;
import org.wpilib.vision.camera.OpenCvLoader;

/** Visualize any serial chain in 3d. */
public class Serial3dVisualization {

    // Line width in the widget, in pixels.
    private static final int LINE_WIDTH = 3;

    // Where the camera is pointing.
    private static final Translation3d CENTER = new Translation3d(0.25, 0, 0);
    // Reverses the rotation.
    private static final Transform3d FLIP = new Transform3d(
            Pose3d.kZero, new Pose3d(0, 0, 0, new Rotation3d(0, 0, Math.PI)));
    // Transforms the camera z to look along -x
    private static final Transform3d LOOKAT = new Transform3d(
            new Translation3d(),
            new Rotation3d(MatBuilder.fill(Nat.N3(), Nat.N3(),
                    0, -1, 0, //
                    0, 0, -1, //
                    1, 0, 0)))
            .inverse();

    private final Supplier<List<Pose3d>> m_arm;

    private final Mechanism2d m_view;
    private final MechanismRoot2d m_root;
    private final MechanismLigament2d m_base;
    private final Map<String, MechanismLigament2d> ligaments = new HashMap<>();

    // coordinates of the camera.
    private double m_cameraRange;
    private double m_pitch;
    private double m_yaw;

    public Serial3dVisualization(Supplier<List<Pose3d>> arm) {
        OpenCvLoader.forceStaticLoad();
        m_arm = arm;

        // size matches the intrinsic matrix
        m_view = new Mechanism2d(200, 200);

        // camera is pointing at the origin in the center: (50, 50)
        m_root = m_view.getRoot("root", 100, 100);

        // base angle is zero (pointing right)
        m_base = new MechanismLigament2d("link", 0, 0, 0, new Color8Bit(Color.BLACK));
        m_root.append(m_base);

        m_cameraRange = 1;
        // pitch up to the camera, i.e. it is above the table.
        m_pitch = -0.615;
        // start behind the arm and to the right
        m_yaw = -2.356;

        SmartDashboard.putData("View", m_view);
    }

    public void periodic() {
        // Orbit in yaw.
        m_yaw += 0.002;
        paintAll();
    }

    void paintAll() {
        Pose3d m_cameraPose = getCameraPose();

        List<Pose3d> pList = m_arm.get();
        paint(m_base, "actual_arm", m_cameraPose, pList, Color.ORANGE_RED);

        List<Pose3d> tList3 = List.of(
                new Pose3d(0, 0, 0, Rotation3d.kZero),
                new Pose3d(0, 0.5, 0, Rotation3d.kZero),
                new Pose3d(0.5, 0.5, 0, Rotation3d.kZero),
                new Pose3d(0.5, -0.5, 0, Rotation3d.kZero),
                new Pose3d(0, -0.5, 0, Rotation3d.kZero),
                new Pose3d(0, 0, 0, Rotation3d.kZero));
        paint(m_base, "tabletop", m_cameraPose, tList3, Color.GRAY);
    }

    Pose3d getCameraPose() {
        // spherical location of the camera; this rotation is not the direction
        // the camera is facing
        Rotation3d r = new Rotation3d(0, m_pitch, m_yaw);
        Translation3d t = new Translation3d(m_cameraRange, 0, 0).rotateBy(r);

        // Rotation facing out
        Pose3d p1 = new Pose3d(t, r);
        // Rotation facing in
        Pose3d p2 = p1.plus(FLIP);
        // Rotate the camera z so it faces in
        Pose3d p3 = p2.plus(LOOKAT);
        // Add an offset to the center of the table
        return new Pose3d(p3.getTranslation().plus(CENTER), p3.getRotation());
    }

    void paint(
            MechanismLigament2d base,
            String name,
            Pose3d cameraPose,
            List<Pose3d> tList,
            Color color) {

        Pair<MatOfPoint2f, List<Color>> points = project(cameraPose, tList, color);
        List<Point> pointList = points.getFirst().toList();
        // System.out.printf("pointlist size %d\n", pointList.size());
        double x0 = 50;
        double y0 = 50;
        double t0 = 0;
        for (int i = 0; i < pointList.size(); ++i) {
            // this is a point in the camera, which is x-left, y-down,
            // looking towards +z,
            // which implies clockwise-positive angles in the xy plane.
            // the mechanism2d widget uses counterclockwise-positive angles.
            // we could invert the xy coordinates, but then we'd have to remember
            // that the "camera" coordinates are unusual.
            // instead, we'll just invert the angle.
            Point p = pointList.get(i);
            Color c = points.getSecond().get(i);

            double dx = p.x - x0;
            double dy = p.y - y0;
            double length = Math.hypot(dx, dy);
            double absoluteAngle = -1.0 * Math.atan2(dy, dx);
            double relativeAngle = absoluteAngle - t0;
            String fullname = String.format("%s_%d", name, i);
            MechanismLigament2d link = ligaments.get(fullname);
            if (link == null) {
                link = new MechanismLigament2d(
                        fullname,
                        length,
                        Math.toDegrees(relativeAngle),
                        LINE_WIDTH,
                        i == 0 ? new Color8Bit(Color.BLACK) : new Color8Bit(c));
                base.append(link);
                base = link;
                ligaments.put(fullname, link);
            } else {
                link.setLength(length);
                link.setAngle(Math.toDegrees(relativeAngle));
            }
            x0 = p.x;
            y0 = p.y;
            t0 = absoluteAngle;
        }
    }

    Pair<MatOfPoint2f, List<Color>> project(
            Pose3d cameraPose, List<Pose3d> tList, Color link) {
        // the extrinsic matrix is the inverse of the camera pose.
        Transform3d extrinsic = new Transform3d(Pose3d.kZero, cameraPose).inverse();

        Mat rvec = getRvec(extrinsic);
        Mat tVec = getTVec(extrinsic);
        Mat kMat = getKMat();

        MatOfDouble dMat = new MatOfDouble(0, 0, 0, 0, 0);
        MatOfPoint2f imagePts2f = new MatOfPoint2f();

        Pair<MatOfPoint3f, List<Color>> objectPts = objectPts(tList, link);
        Calib3d.projectPoints(objectPts.getFirst(), rvec, tVec, kMat, dMat, imagePts2f);
        return new Pair<>(imagePts2f, objectPts.getSecond());
    }

    private Mat getTVec(Transform3d extrinsic) {
        Translation3d t = extrinsic.getTranslation();
        Mat tVec = Mat.zeros(3, 1, CvType.CV_64F);
        tVec.put(0, 0, t.getX(), t.getY(), t.getZ());
        return tVec;
    }

    private Mat getKMat() {
        Mat kMat = Mat.zeros(3, 3, CvType.CV_64F);
        kMat.put(0, 0,
                100.0, 0.0, 50.0,
                0.0, 100.0, 50.0,
                0.0, 0.0, 1.0);
        return kMat;
    }

    private Mat getRvec(Transform3d extrinsic) {
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
     * Make a point in link color for each pose, and also the basis vectors
     * using x=red, y=green, z=blue
     */
    private Pair<MatOfPoint3f, List<Color>> objectPts(List<Pose3d> tList, Color link) {
        List<Point3> pList = new ArrayList<>();
        List<Color> cList = new ArrayList<>();
        for (Pose3d p : tList) {
            Translation3d t = p.getTranslation();
            Rotation3d R = p.getRotation();
            pList.add(point(t));
            cList.add(link);
            // Show the basis vectors
            basis(pList, cList, t, R, new Translation3d(1, 0, 0), Color.RED);
            basis(pList, cList, t, R, new Translation3d(0, 1, 0), Color.GREEN);
            basis(pList, cList, t, R, new Translation3d(0, 0, 1), Color.BLUE);
        }
        return new Pair<>(new MatOfPoint3f(pList.toArray(new Point3[0])), cList);
    }

    /** Add a little line showing a basis vector. */
    private void basis(
            List<Point3> pList,
            List<Color> cList,
            Translation3d t,
            Rotation3d R,
            Translation3d yt,
            Color basis) {
        Translation3d a2 = new Translation3d(yt.rotateBy(R).toVector()).times(0.03);
        // A point on the end of the basis vector.
        pList.add(point(t.plus(a2)));
        cList.add(basis);
        // Back to the origin.
        pList.add(point(t));
        cList.add(basis);

    }

    private Point3 point(Translation3d t) {
        return new Point3(t.getX(), t.getY(), t.getZ());
    }
}
