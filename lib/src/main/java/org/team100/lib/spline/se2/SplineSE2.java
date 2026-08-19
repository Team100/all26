package org.team100.lib.spline.se2;

import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.PathUtil;
import org.team100.lib.path.se2.PathSE2Entry;
import org.team100.lib.path.se2.PathSE2Parameter;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.spline.r1.SplineR1;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.numbers.N2;

/**
 * Spline in the SE(2) manifold, the space Pose2d lives in.
 * 
 * The three dimensions (x, y, heading) of the Pose2d, p, are independent
 * one-dimensional splines, with respect to a parameter s∈[0,1].
 */
public class SplineSE2 implements ISplineSE2 {
    private static final double DEFAULT_SCALE = 1;
    private static final boolean DEBUG = false;

    private final SplineR1 m_x;
    private final SplineR1 m_y;
    private final SplineR1 m_heading;
    /**
     * Offset for heading spline: the heading spline doesn't include the
     * starting point in order to correctly handle wrapping.
     */
    private final Rotation2d m_heading0;

    /**
     * Specify the magic number you want: this scales the derivatives at the
     * endpoints, i.e. how "strongly" the derivative affects the curve. High magic
     * number means low curvature at the endpoint.
     * 
     * Previously, the theta endpoint derivatives were the average rate, which
     * yielded paths with a lot of rotation at the last second. Typically this isn't
     * what you want: you're approaching some target, and rotation is disruptive to
     * everything: vision, actuation, everything.
     *
     * Instead, we now use the "course" to specify the whole SE(2) direction, so if
     * you want rotation at the end, you can say that, and if you want no rotation
     * at the end, you can say that too.
     * 
     * All second derivatives are zero, and we don't try to change this anymore with
     * optimization. Optimization just doesn't help very much, and it's a pain when
     * it behaves strangely.
     * 
     * To avoid confusion, the parameter should always be called "s".
     * 
     * @param p0 starting pose
     * @param p1 ending pose
     */
    public SplineSE2(WaypointSE2 p0, WaypointSE2 p1) {
        // Translation distance in the xy plane.
        double distance = Metrics.translationalDistance(p0.pose(), p1.pose());
        if (distance < 1e-6)
            throw new IllegalArgumentException("splines must cover xy distance");

        if (DEBUG)
            System.out.printf("distance %f\n", distance);
        double scale0 = p0.scale() * distance;
        double scale1 = p1.scale() * distance;

        if (DEBUG) {
            System.out.printf("scale %f %f\n", scale0, scale1);
        }

        // Endpoints:
        double x0 = p0.pose().getTranslation().getX();
        double x1 = p1.pose().getTranslation().getX();
        double y0 = p0.pose().getTranslation().getY();
        double y1 = p1.pose().getTranslation().getY();
        // To avoid 180 degrees, heading uses an offset
        m_heading0 = p0.pose().getRotation();
        double delta = p1.pose().getRotation().minus(p0.pose().getRotation()).getRadians();

        // First derivatives are the course:
        double dx0 = p0.course().x * scale0;
        double dx1 = p1.course().x * scale1;
        double dy0 = p0.course().y * scale0;
        double dy1 = p1.course().y * scale1;
        double dtheta0 = p0.course().theta * scale0;
        double dtheta1 = p1.course().theta * scale1;

        // Second derivatives are zero:
        double ddx0 = 0;
        double ddx1 = 0;
        double ddy0 = 0;
        double ddy1 = 0;
        double ddtheta0 = 0;
        double ddtheta1 = 0;

        m_x = SplineR1.get(x0, x1, dx0, dx1, ddx0, ddx1);
        m_y = SplineR1.get(y0, y1, dy0, dy1, ddy0, ddy1);
        m_heading = SplineR1.get(0.0, delta, dtheta0, dtheta1, ddtheta0, ddtheta1);
    }

    @Override
    public WaypointSE2 waypoint(double s) {
        return new WaypointSE2(pose(s), course(s), DEFAULT_SCALE);
    }

    @Override
    public Pose2d pose(double s) {
        return new Pose2d(new Translation2d(x(s), y(s)), heading(s));
    }

    @Override
    public PathSE2Entry entry(double s) {
        return new PathSE2Entry(parameter(s), point(s));
    }

    public PathSE2Parameter parameter(double s) {
        return new PathSE2Parameter(this, s);
    }

    public PathSE2Point point(double s) {
        return new PathSE2Point(waypoint(s), K(s));
    }

    ////////////////////////////////////////////////////////////
    ///
    /// position, p

    double x(double s) {
        return m_x.getPosition(s);
    }

    double y(double s) {
        return m_y.getPosition(s);
    }

    Rotation2d heading(double s) {
        return m_heading0.rotateBy(Rotation2d.fromRadians(m_heading.getPosition(s)));
    }

    ////////////////////////////////////////////////////////////
    ///
    /// first derivative, dp/ds or pprime

    double dx(double s) {
        return m_x.getVelocity(s);
    }

    private double dy(double s) {
        return m_y.getVelocity(s);
    }

    public double dheading(double s) {
        return m_heading.getVelocity(s);
    }

    private DirectionSE2 course(double s) {
        double dx = dx(s);
        double dy = dy(s);
        double dheading = dheading(s);
        if (DEBUG)
            System.out.printf("%f %f %f\n", dx, dy, dheading);
        try {
            return new DirectionSE2(dx, dy, dheading);
        } catch (IllegalArgumentException ex) {
            throw new IllegalArgumentException(
                    String.format("bad direction for s=%f, dx=%f, dy=%f, dheading=%f",
                            s, dx, dy, dheading),
                    ex);
        }
    }

    /** Magnitude of translational part of dp/ds */
    private double pprimeTranslationNorm(double s) {
        double dx = dx(s);
        double dy = dy(s);
        return Math.hypot(dx, dy);
    }

    /** Heading angular velocity with respect to translation. */
    double headingRate(double s) {
        double dheading = dheading(s);
        double v = pprimeTranslationNorm(s);
        return dheading / v;
    }

    ////////////////////////////////////////////////////////////
    ///
    /// second derivative, d^2q/ds^2 or pprimeprime

    double ddx(double s) {
        return m_x.getAcceleration(s);
    }

    private double ddy(double s) {
        return m_y.getAcceleration(s);
    }

    public double ddheading(double s) {
        return m_heading.getAcceleration(s);
    }

    /**
     * Scalar curvature, $\kappa$, is the norm of the curvature vector, signed (CCW
     * positive) with respect to the tangent vector.
     * 
     * see MATH.md.
     */
    public double curvature(double s) {
        return PathUtil.kappaSigned(T(s), K(s));
    }

    /**
     * Curvature vector is the change in motion direction per distance traveled.
     * rad/m.
     * 
     * see MATH.md
     */
    @Override
    public Vector<N2> K(double s) {
        return PathUtil.K(rprime(s), rprimeprime(s));
    }

    /**
     * Tangent vector is a unit vector in the direction of motion.
     * 
     * see MATH.md
     */
    Vector<N2> T(double s) {
        return PathUtil.T(rprime(s));
    }

    /**
     * dr/ds, derivative of position with respect to parameter, s
     */
    public Vector<N2> rprime(double s) {
        return VecBuilder.fill(dx(s), dy(s));
    }

    /**
     * d^2r/ds^2, second derivative of position with respect to parameter, s
     */
    public Vector<N2> rprimeprime(double s) {
        return VecBuilder.fill(ddx(s), ddy(s));
    }

    ////////////////////////////////////////////////////////////

    @Override
    public String toString() {
        return "HolonomicSplineSE2 [m_x=" + m_x
                + ", m_y=" + m_y
                + ", m_theta=" + m_heading
                + ", m_r0=" + m_heading0 + "]";
    }
}