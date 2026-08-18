package org.team100.lib.localization;

import java.util.function.Consumer;
import java.util.function.UnaryOperator;

import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.sensor.gyro.SimulatedGyro;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;
import org.team100.lib.visualization.RobotPoseVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Container for aspects of ground truth for simulation.
 * 
 * To correctly simulate the influence of vision and gyro drift on robot
 * rotation, we need to track the "ground truth" of the robot separately from
 * the simulated measurements (which include drift).
 */
public class GroundTruth {
    private final Runnable m_simulatedTagDetector;
    private final Consumer<Pose2d> m_groundTruthResetter;
    private final Runnable m_groundTruthViz;

    public GroundTruth(
            LoggerFactory fieldLogger,
            LoggerFactory logger,
            SwerveKinodynamics m_swerveKinodynamics,
            SwerveModuleCollection m_modules,
            AprilTagFieldLayoutWithCorrectOrientation layout) {
        if (RobotBase.isReal()) {
            // Real robots get an empty simulated tag detector.
            m_groundTruthViz = () -> {
            };
            m_simulatedTagDetector = () -> {
            };
            m_groundTruthResetter = (p) -> {
            };
        } else {
            // This is all for simulation only.
            final LoggerFactory simLog = logger.name("Simulation");

            // Ground-truth simulated gyro does not drift at all.
            SimulatedGyro groundTruthGyro = new SimulatedGyro(simLog,
                    m_swerveKinodynamics, m_modules, 0);

            // History of ground-truth poses is based only on odometry.
            SwerveHistory groundTruthHistory = new SwerveHistory(
                    simLog,
                    m_swerveKinodynamics,
                    0.2,
                    groundTruthGyro.getYawNWU(),
                    VariableR1.fromStdDev(0, 1),
                    m_modules.positions(),
                    Pose2d.kZero,
                    IsotropicNoiseSE2.high(),
                    Takt.get());

            // Read positions and ground truth gyro (which are perfectly consistent) and
            // maintain the ground truth history.
            OdometryUpdater groundTruthUpdater = new OdometryUpdater(
                    simLog, m_swerveKinodynamics, groundTruthGyro,
                    groundTruthHistory, m_modules::positions,
                    UnaryOperator.identity());
            m_groundTruthResetter = (p) -> groundTruthUpdater.reset(p, IsotropicNoiseSE2.high());

            GroundTruthCache groundTruthCache = new GroundTruthCache(
                    groundTruthUpdater, groundTruthHistory);

            // Visualization of the simulated "ground truth" of the robot pose.
            m_groundTruthViz = new RobotPoseVisualization(
                    fieldLogger, () -> groundTruthCache.apply(Takt.get()).pose(), "ground truth");

            // Simulated camera uses the ground truth because the real cameras are not aware
            // of the pose estimate.
            m_simulatedTagDetector = SimulatedTagDetector.get(
                    layout, groundTruthHistory);
        }
    }

    public void resetPose(Pose2d p) {
        m_groundTruthResetter.accept(p);
    }

    public void periodic() {
        // publish the simulated tag sightings.
        m_simulatedTagDetector.run();
        // publish ground truth pose
        if (m_groundTruthViz != null)
            m_groundTruthViz.run();
    }
}
