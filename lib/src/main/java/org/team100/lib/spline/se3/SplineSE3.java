package org.team100.lib.spline.se3;

import org.team100.lib.geometry.se3.DirectionSE3;
import org.team100.lib.geometry.se3.WaypointSE3;
import org.team100.lib.path.PathUtil;
import org.team100.lib.path.se3.PathSE3Entry;
import org.team100.lib.path.se3.PathSE3Parameter;
import org.team100.lib.path.se3.PathSE3Point;
import org.team100.lib.spline.r1.SplineR1;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.numbers.N3;

/**
 * Holonomic spline in the SE(3) manifold.
 * 
 * The six dimensions (x, y, z, roll, pitch, yaw) of the Pose3d, p, are
 * independent one-dimensional splines, with respect to a parameter s∈[0,1].
 */
public class SplineSE3 {
    private static final double DEFAULT_SCALE = 1;
    private static final boolean DEBUG = false;

    // these are for position
    private final SplineR1 m_x;
    private final SplineR1 m_y;
    private final SplineR1 m_z;

    // these are for heading
    private final SplineR1 m_roll;
    private final SplineR1 m_pitch;
    private final SplineR1 m_yaw;
    private final Rotation2d m_roll0;
    private final Rotation2d m_pitch0;
    private final Rotation2d m_yaw0;

    public SplineSE3(WaypointSE3 p0, WaypointSE3 p1) {
        this(p0, p1, 1.2, 1.2);
    }

    public SplineSE3(WaypointSE3 p0, WaypointSE3 p1, double mN0, double mN1) {
        double distance = p0.pose().getTranslation().getDistance(p1.pose().getTranslation());
        double scale0 = mN0 * distance;
        double scale1 = mN1 * distance;

        // endpoints
        double x0 = p0.pose().getTranslation().getX();
        double x1 = p1.pose().getTranslation().getX();
        double y0 = p0.pose().getTranslation().getY();
        double y1 = p1.pose().getTranslation().getY();
        double z0 = p0.pose().getTranslation().getZ();
        double z1 = p1.pose().getTranslation().getZ();

        // first derivatives are the course
        double dx0 = p0.course().x * scale0;
        double dx1 = p1.course().x * scale1;
        double dy0 = p0.course().y * scale0;
        double dy1 = p1.course().y * scale1;
        double dz0 = p0.course().z * scale0;
        double dz1 = p1.course().z * scale1;

        // second derivatives are zero
        double ddx0 = 0;
        double ddx1 = 0;
        double ddy0 = 0;
        double ddy1 = 0;
        double ddz0 = 0;
        double ddz1 = 0;

        m_x = SplineR1.get(x0, x1, dx0, dx1, ddx0, ddx1);
        m_y = SplineR1.get(y0, y1, dy0, dy1, ddy0, ddy1);
        m_z = SplineR1.get(z0, z1, dz0, dz1, ddz0, ddz1);

        Rotation3d r0 = p0.pose().getRotation();
        Rotation3d r1 = p1.pose().getRotation();
        if (DEBUG) {
            System.out.printf("r0 %f %f %f\n", r0.getX(), r0.getY(), r0.getZ());
            System.out.printf("r1 %f %f %f\n", r1.getX(), r1.getY(), r1.getZ());
        }
        if (DEBUG) {
            Rotation3d i0 = r0.interpolate(r1, 0);
            System.out.printf("interp 0.0 %f %f %f\n", i0.getX(), i0.getY(), i0.getZ());
            Rotation3d i1 = r0.interpolate(r1, 0.5);
            System.out.printf("interp 0.5 %f %f %f\n", i1.getX(), i1.getY(), i1.getZ());
            Rotation3d i2 = r0.interpolate(r1, 1);
            System.out.printf("interp 1.0 %f %f %f\n", i2.getX(), i2.getY(), i2.getZ());
        }
        if (DEBUG) {
            System.out.printf("r0 %f %f %f\n", r0.getX(), r0.getY(), r0.getZ());
            Rotation3d r0inv = r0.inverse();
            System.out.printf("r0inv %f %f %f\n", r0inv.getX(), r0inv.getY(), r0inv.getZ());
        }

        m_roll0 = new Rotation2d(r0.getX());
        m_pitch0 = new Rotation2d(r0.getY());
        m_yaw0 = new Rotation2d(r0.getZ());

        if (DEBUG) {
            System.out.printf("r0 roll %5.3f pitch %5.3f yaw %5.3f\n", r0.getX(), r0.getY(), r0.getZ());
            System.out.printf("r1 roll %5.3f pitch %5.3f yaw %5.3f\n", r1.getX(), r1.getY(), r1.getZ());
        }
        // "minus" does something strange
        // see https://github.com/wpilibsuite/allwpilib/issues/8523
        // so we do COMPONENTWISE instead.
        // Rotation3d headingDelta = r1.minus(r0);

        double rollDelta = MathUtil.angleModulus(r1.getX() - r0.getX());
        double pitchDelta = MathUtil.angleModulus(r1.getY() - r0.getY());
        double yawDelta = MathUtil.angleModulus(r1.getZ() - r0.getZ());
        if (DEBUG) {
            System.out.printf("rollDelta %5.3f pitchDelta %5.3f yawDelta %5.3f\n",
                    rollDelta, pitchDelta, yawDelta);
        }

        // first derivatives are the course
        double droll0 = p0.course().roll * mN0;
        double droll1 = p1.course().roll * mN1;
        double dpitch0 = p0.course().pitch * mN0;
        double dpitch1 = p1.course().pitch * mN1;
        double dyaw0 = p0.course().yaw * mN0;
        double dyaw1 = p1.course().yaw * mN1;

        // second derivatives are zero
        double ddroll0 = 0;
        double ddroll1 = 0;
        double ddpitch0 = 0;
        double ddpitch1 = 0;
        double ddyaw0 = 0;
        double ddyaw1 = 0;

        m_roll = SplineR1.get(0.0, rollDelta, droll0, droll1, ddroll0, ddroll1);
        m_pitch = SplineR1.get(0.0, pitchDelta, dpitch0, dpitch1, ddpitch0, ddpitch1);
        m_yaw = SplineR1.get(0.0, yawDelta, dyaw0, dyaw1, ddyaw0, ddyaw1);
    }

    public PathSE3Point sample(double s) {
        Vector<N3> K = K(s);
        Vector<N3> H = headingRate(s);
        return new PathSE3Point(waypoint(s), K, H);
    }

    public WaypointSE3 waypoint(double s) {
        return new WaypointSE3(pose(s), course(s), DEFAULT_SCALE);
    }

    public Pose3d pose(double s) {
        return new Pose3d(new Translation3d(x(s), y(s), z(s)), heading(s));
    }

    public PathSE3Entry entry(double s) {
        return new PathSE3Entry(new PathSE3Parameter(this, s), sample(s));
    }

    ////////////////////////////////////////////////////////////
    ///
    /// position, p

    private double x(double s) {
        return m_x.getPosition(s);
    }

    private double y(double s) {
        return m_y.getPosition(s);
    }

    private double z(double s) {
        return m_z.getPosition(s);
    }

    private Rotation3d heading(double s) {
        return new Rotation3d(
                m_roll0.plus(new Rotation2d(m_roll.getPosition(s))).getRadians(),
                m_pitch0.plus(new Rotation2d(m_pitch.getPosition(s))).getRadians(),
                m_yaw0.plus(new Rotation2d(m_yaw.getPosition(s))).getRadians());
    }

    ////////////////////////////////////////////////////////////
    ///
    /// first derivative, dp/ds or pprime

    private double dx(double s) {
        return m_x.getVelocity(s);
    }

    private double dy(double s) {
        return m_y.getVelocity(s);
    }

    private double dz(double s) {
        return m_z.getVelocity(s);
    }

    private double droll(double s) {
        return m_roll.getVelocity(s);
    }

    private double dpitch(double s) {
        return m_pitch.getVelocity(s);
    }

    private double dyaw(double s) {
        return m_yaw.getVelocity(s);
    }

    private DirectionSE3 course(double s) {
        double dx = dx(s);
        double dy = dy(s);
        double dz = dz(s);
        double dr = droll(s);
        double dp = dpitch(s);
        double dyaw = dyaw(s);
        return new DirectionSE3(dx, dy, dz, dr, dp, dyaw);
    }

    /** Magnitude of translational part of dp/ds */
    private double pprimeTranslationNorm(double s) {
        double dx = dx(s);
        double dy = dy(s);
        double dz = dz(s);
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    /** Heading angular velocity with respect to s. */
    public Vector<N3> headingRate(double s) {
        // drad/ds
        double droll = droll(s);
        double dpitch = dpitch(s);
        double dyaw = dyaw(s);
        // dm/ds
        double v = pprimeTranslationNorm(s);
        // drad/ds / dm/ds = drad/dm
        return VecBuilder.fill(droll / v, dpitch / v, dyaw / v);
    }

    ////////////////////////////////////////////////////////////
    ///
    /// second derivative, d^2q/ds^2 or pprimeprime

    private double ddx(double s) {
        return m_x.getAcceleration(s);
    }

    private double ddy(double s) {
        return m_y.getAcceleration(s);
    }

    private double ddz(double s) {
        return m_z.getAcceleration(s);
    }

    /**
     * Scalar curvature, $\kappa$, is the norm of the curvature vector.
     * 
     * see MATH.md.
     */
    double curvature(double s) {
        return K(s).norm();
    }

    /**
     * Curvature vector is the change in motion direction per distance traveled.
     * rad/m.
     * 
     * see MATH.md
     */
    public Vector<N3> K(double s) {
        return PathUtil.K(rprime(s), rprimeprime(s));
    }

    /**
     * dr/ds, position derivative with respect to parameter, s.
     */
    private Vector<N3> rprime(double s) {
        return VecBuilder.fill(dx(s), dy(s), dz(s));
    }

    /**
     * d^2r/ds^2, second derivative of position with respect to parameter, s
     */
    private Vector<N3> rprimeprime(double s) {
        return VecBuilder.fill(ddx(s), ddy(s), ddz(s));
    }

    ////////////////////////////////////////////////////////////

    /**
     * Print samples for testing.
     * 
     * Uses python format, for use with this Google Colab python notebook.
     * 
     * https://colab.research.google.com/drive/1iZU72lggE4oH551WXamc-9_Mh_1zR0kV#scrollTo=IFKxOJBoXLEr
     * 
     * It's quite slow, but it's very simple.
     */
    public void dump() {
        Translation3d arrow = new Translation3d(0.1, 0, 0);
        for (double s = 0; s <= 1; s += 0.05) {
            Rotation3d h = heading(s);
            if (DEBUG)
                System.out.printf("heading %5.3f %5.3f %5.3f\n", h.getX(), h.getY(), h.getZ());
            Translation3d t = arrow.rotateBy(h);
            System.out.printf("[%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f],\n",
                    x(s), y(s), z(s), t.getX(), t.getY(), t.getZ());
        }
    }

}
