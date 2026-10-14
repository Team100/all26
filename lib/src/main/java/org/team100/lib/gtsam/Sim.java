package org.team100.lib.gtsam;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.gtsam.config.CameraConfig;
import org.team100.lib.gtsam.field.FieldMap;
import org.team100.lib.gtsam.pose_estimator.BetweenGyro;
import org.team100.lib.gtsam.pose_estimator.Gyro;
import org.team100.lib.gtsam.pose_estimator.Odometry;
import org.team100.lib.gtsam.pose_estimator.Prior;
import org.team100.lib.gtsam.pose_estimator.Solver;
import org.team100.lib.gtsam.pose_estimator.Vision;
import org.team100.lib.gtsam.simulation.SimulatedCamera;
import org.team100.lib.gtsam.simulation.SimulatedGyro;
import org.team100.lib.gtsam.simulation.SimulatedOdometry;
import org.team100.lib.gtsam.simulation.SimulatedRobot;
import org.team100.lib.gtsam.util.Geometry;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.FieldObject2d;
import org.wpilib.smartdashboard.SmartDashboard;

import gtsam.Key;
import gtsam.Marginals;
import gtsam.Matrix;
import gtsam.NonlinearFactorGraph;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose2;
import gtsam.Values;
import gtsam.Vector;
import gtsam.Vector1;
import gtsam.Vector3;
import gtsam.noiseModel.Diagonal;

/**
 * Outer simulation loop. Call "run" periodically.
 */
public class Sim {
    private static final boolean DEBUG = false;

    public final Solver m_solver;
    public final Field2d m_field;
    public final List<Point3> m_landmarks;

    // simulated measurements
    public final SimulatedOdometry m_simulatedOdometry;
    public final SimulatedRobot m_simulatedRobot;
    public final SimulatedCamera m_simulatedCamera;
    public final SimulatedGyro m_simulatedGyro;

    // factors
    public final Vision m_vision;
    // private final Gyro m_gyro;
    public final BetweenGyro m_betweenGyro;
    public final Odometry m_odometry;
    public final Prior m_prior;

    public Pose2d m_groundTruthPose;
    /** Estimate from the solver. */
    public Pose2 m_estimatedPose;
    public int m_loopCount;

    // the verbosity here is to trap the exception.
    public Sim() throws Throwable {

        int lagMicroseconds = 100000;
        m_solver = new Solver(lagMicroseconds);

        m_estimatedPose = new Pose2();

        //
        // LANDMARKS
        //
        List<Point3> tag = new FieldMap().get(0);
        m_landmarks = List.of(tag.get(0), tag.get(1), tag.get(2), tag.get(3));
        FieldMap fieldMap = new FieldMap();
        m_field = new Field2d();
        m_field.getObject("tag0").setPose(new Pose2d(tag.get(0).x(), tag.get(0).y(), new Rotation2d(0)));
        m_field.getObject("tag1").setPose(new Pose2d(tag.get(1).x(), tag.get(1).y(), new Rotation2d(0)));
        m_field.getObject("tag2").setPose(new Pose2d(tag.get(2).x(), tag.get(2).y(), new Rotation2d(0)));
        m_field.getObject("tag3").setPose(new Pose2d(tag.get(3).x(), tag.get(3).y(), new Rotation2d(0)));

        CameraConfig conf = new CameraConfig();

        //
        // SIMULATED MEASUREMENTS
        //
        m_simulatedRobot = new SimulatedRobot();
        Pose2d initial = m_simulatedRobot.pose(0);
        m_simulatedCamera = new SimulatedCamera(m_landmarks, conf);
        m_simulatedGyro = new SimulatedGyro(true);

        //
        // FACTORS
        //
        m_vision = new Vision(m_solver, conf);
        // m_gyro = new Gyro(m_solver);
        m_betweenGyro = new BetweenGyro(m_solver);
        m_odometry = new Odometry(m_solver);
        m_prior = new Prior(m_solver);

        // Initial pose.
        Pose2 p0 = new Pose2(0, 0, 0);
        Key x0 = Key.X(0);
        m_solver.addVariable(x0, 0, p0);
        m_prior.add(x0, p0, Diagonal.Sigmas(new Vector3(100, 100, 100)));

        // Initial gyro bias.

        Key b0 = Key.B(0);
        m_solver.addVariable(b0, 0, 0);
        // // try a very low bias prior
        m_prior.add(b0, 0, Diagonal.Sigmas(new Vector1(0.001)));
        m_betweenGyro.add(0, m_simulatedGyro.yaw(0, initial));

        // Record the initial timestamp and positions.
        m_simulatedOdometry = new SimulatedOdometry(fieldMap, initial);

        m_odometry.add(0, m_simulatedOdometry.positions(initial));

        m_loopCount = 1;

        SmartDashboard.putData("Field", m_field);
    }

    /** Constructor without exception */
    public static Sim make() {
        try {
            return new Sim();
        } catch (Throwable e) {
            throw new RuntimeException(e);
        }
    }

    public void run() {
        try {
            run0();
        } catch (Throwable e) {
            throw new RuntimeException(e);
        }
    }

    public void run0() throws Throwable {

        if (DEBUG)
            System.out.printf("==> Loop count %d\n", m_loopCount);
        SmartDashboard.putNumber("i", m_loopCount);

        // Nanosecond timer to see how long the solver takes.
        long t0_ns = System.nanoTime();

        // Current simulation time in microseconds.
        long t1_us = 20000 * m_loopCount;

        // Compute ground truth and plot it.
        m_groundTruthPose = m_simulatedRobot.pose(t1_us);
        m_field.getObject("gt").setPose(m_groundTruthPose);

        if (DEBUG)
            System.out.println("==> Initial value is the previous estimate.");
        Key x1 = Key.X(t1_us);
        m_solver.addVariable(x1, t1_us, m_estimatedPose);

        if (DEBUG)
            System.out.println("==> Add odometry factors.");

        applyOdometry(t1_us, m_groundTruthPose);

        if (DEBUG)
            System.out.println("==> Add gyro factors.");

        applyBetweenGyro(t1_us, m_groundTruthPose);

        if (DEBUG)
            System.out.println("==> Add camera factors.");

        applyCamera(t1_us, m_groundTruthPose);

        if (DEBUG)
            System.out.println("==> Run the solver.");
        m_solver.update();

        if (DEBUG)
            System.out.println("==> Print the graph.");
        NonlinearFactorGraph factors = m_solver.getFactors();
        if (DEBUG)
            factors.print("factors");

        if (DEBUG)
            System.out.println("==> Print the values.");
        Values values = m_solver.result();
        if (DEBUG)
            values.print("values");

        if (DEBUG)
            System.out.println("==> Print factors and errors.");
        if (DEBUG)
            factors.printErrors(values, "factors and errors");

        if (DEBUG)
            System.out.println("==> Log a little about the iteration.");
        logET(t0_ns);

        if (DEBUG)
            System.out.println("==> Retrieve the estimated pose.");
        m_estimatedPose = m_solver.mean_pose2(x1);
        if (DEBUG)
            System.out.printf("==> Estimated pose %s", m_estimatedPose.toString());

        if (DEBUG)
            System.out.println("==> Show the estimate, and errors.");
        plotEstimatedPose(m_groundTruthPose);
        if (DEBUG)
            System.out.println("==> Show samples on the field.");
        plotSamples(t1_us);

        if (DEBUG)
            System.out.println("==> Show the estimated bias.");

        double b = m_solver.mean_double(Key.B(t1_us));
        if (DEBUG)
            System.out.printf("==> Estimated bias is %f\n", b);
        SmartDashboard.putNumber("bias", b);

        Vector poseSigma = m_solver.sigma_pose2(x1);
        double x = poseSigma.at(0);
        double y = poseSigma.at(1);
        double s = poseSigma.at(2);
        if (DEBUG)
            System.out.printf("==> Pose sigma %f %f %f\n", x, y, s);
        SmartDashboard.putNumber("pose sigma x (m)", x);
        SmartDashboard.putNumber("pose sigma y (m)", y);
        SmartDashboard.putNumber("pose sigma (rad)", s);

        if (DEBUG)
            System.out.println("==> Get the marginals");
        Marginals m = m_solver.marginal_covariance();
        if (DEBUG)
            System.out.println("print the marginals");
        if (DEBUG)
            m.print("marginals");

        if (DEBUG)
            System.out.println("==> Find the bias sigma 2");
        // this is the step that fails,
        // so maybe just don't do that?
        Matrix s1 = m.marginalCovariance(Key.B(t1_us));
        if (DEBUG)
            System.out.println("==> Find the bias sigma 3");
        Vector biasSigma = s1.diagonal_cwiseSqrt();
        if (DEBUG)
            System.out.println("==> Find the bias sigma 4");
        double bs = biasSigma.at(0);
        if (DEBUG)
            System.out.printf("==> Bias sigma is %f\n", bs);
        SmartDashboard.putNumber("bias sigma (rad)", bs);

        ++m_loopCount;

    }

    /** Plot the estimated pose and the error from ground truth. */
    private void plotEstimatedPose(Pose2d groundTruthPose) throws Throwable {
        Pose2d estPose2d = new Pose2d(m_estimatedPose.x(), m_estimatedPose.y(),
                new Rotation2d(m_estimatedPose.theta()));
        m_field.setRobotPose(estPose2d);
        logErr(groundTruthPose, estPose2d);
    }

    private void logET(long t0_ns) throws Throwable {
        long t1_ns = System.nanoTime();
        long et_ns = t1_ns - t0_ns;
        SmartDashboard.putNumber("et (ms)", (double) et_ns * 1e-6);
        SmartDashboard.putNumber("size", m_solver.result_size());
    }

    /**
     * Log the estimation error.
     */
    private void logErr(Pose2d gtPose2d, Pose2d estPose2d) {
        Transform2d poseErr = estPose2d.minus(gtPose2d);
        SmartDashboard.putNumber("err_x (m)", poseErr.getX());
        SmartDashboard.putNumber("err_y (m)", poseErr.getY());
        SmartDashboard.putNumber("err_theta (rad)", poseErr.getRotation().getRadians());
    }

    /**
     * Plot some samples around the mean.
     */
    private void plotSamples(long t1_us) throws Throwable {
        int N = 10;
        List<Pose2d> samples = new ArrayList<>();
        for (int i = 0; i < N; ++i) {
            Pose2 sample = m_solver.sample_Pose2(Key.X(t1_us));
            Pose2d wSample = Geometry.toPose2d(sample);
            samples.add(wSample);
        }
        FieldObject2d o = m_field.getObject("samples");
        o.setPoses(samples);
    }

    // private void applyGyro(long t1_us, Pose2d gtPose2d) throws Throwable {
    // double measurement = m_simulatedGyro.yaw(t1_us, gtPose2d);
    // m_gyro.add(t1_us, measurement);
    // }

    void applyBetweenGyro(long t1_us, Pose2d gtPose2d) throws Throwable {
        double measurement = m_simulatedGyro.yaw(t1_us, gtPose2d);
        Key b = Key.B(t1_us);
        m_solver.addVariable(b, t1_us, 0);
        // try a very loose prior? this does not help.
        // m_prior.add(b, 0, Diagonal.Sigmas(new Vector1(1)));
        m_betweenGyro.add(t1_us, measurement);
    }

    void applyOdometry(long t1_us, Pose2d gtPose2d) throws Throwable {
        SwerveModulePositions positions = m_simulatedOdometry.positions(gtPose2d);
        m_odometry.add(t1_us, positions);
    }

    /** Retrieve simulated camera measurements and apply them to the graph. */
    void applyCamera(long t1_us, Pose2d gtPose2d) throws Throwable {
        List<Point2> measurements = m_simulatedCamera.pixels(gtPose2d);
        if (m_landmarks.size() != measurements.size())
            return;
        for (int i = 0; i < m_landmarks.size(); ++i) {
            Point3 landmark = m_landmarks.get(i);
            Point2 measurement = measurements.get(i);
            m_vision.add(t1_us, landmark, measurement);
        }
    }

}
