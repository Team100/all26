package org.team100.lib.localization;

import java.util.function.UnaryOperator;

import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.sensor.gyro.SimulatedGyro;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.targeting.SimulatedTargetWriter;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;
import org.team100.lib.visualization.RobotPoseVisualization;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;

/**
 * Container for aspects of ground truth for simulation.
 * 
 * To correctly simulate the influence of vision and gyro drift on robot
 * rotation, we need to track the "ground truth" of the robot separately from
 * the simulated measurements (which include drift).
 */
public class GroundTruth {
    private final SwerveModuleCollection m_modules;
    private final SimulatedGyro m_gyro;
    private final SwerveHistory m_history;
    private final OdometryUpdater m_odometry;
    private final RobotPoseVisualization m_viz;
    private final SimulatedTagCornerDetector m_tagSim;
    private final SimulatedTargetWriter m_targetSim;

    public GroundTruth(
            LoggerFactory fieldLogger,
            LoggerFactory logger,
            SwerveKinodynamics kinodynamics,
            SwerveModuleCollection modules,
            AprilTagFieldLayoutWithCorrectOrientation layout) {
        LoggerFactory simLog = logger.name("Simulation");
        m_modules = modules;
        // Ground-truth simulated gyro does not drift at all.
        m_gyro = new SimulatedGyro(simLog,
                kinodynamics, m_modules, 0);

        // History of ground-truth poses is based only on odometry.
        m_history = new SwerveHistory(
                simLog,
                kinodynamics,
                0.2,
                m_gyro.getYawNWU(),
                VariableR1.fromStdDev(0, 1),
                m_modules.positions(),
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                Takt.get());

        // Read positions and ground truth gyro (which are perfectly consistent) and
        // maintain the ground truth history.
        m_odometry = new OdometryUpdater(
                simLog, kinodynamics, m_gyro,
                m_history, m_modules::positions,
                UnaryOperator.identity(), true);

        GroundTruthCache groundTruthCache = new GroundTruthCache(
                m_odometry, m_history);

        // Visualization of the simulated "ground truth" of the robot pose.
        m_viz = new RobotPoseVisualization(
                fieldLogger, () -> groundTruthCache.apply(Takt.get()).pose(), "ground truth");

        // Simulated camera uses the ground truth because the real cameras are not aware
        // of the pose estimate.
        // m_simulatedTagDetector = SimulatedTagDetector.get(
        // layout, groundTruthHistory);
        m_tagSim = SimulatedTagCornerDetector.get(
                layout, m_history);
        m_targetSim = SimulatedTargetWriter.get(simLog, m_history);
    }

    /**
     * Reset the pose with roughly infinite uncertainty, so the following update
     * will have a large effect.
     */
    public void resetPose(Pose2d pose) {
        if (RobotBase.isReal() && !Experiments.INSTANCE.enabled(Experiment.SimulateCameras)) {
            // Real robot, but without simulated cameras.
            return;
        }
        m_history.reset(
                m_modules.positions(),
                pose,
                IsotropicNoiseSE2.high(),
                Takt.get(),
                m_gyro.getYawNWU(),
                VariableR1.fromVariance(0, 1));
    }

    /**
     * Show the simulated tags and targets, and the ground-truth robot pose.
     */
    public void periodic() {
        if (RobotBase.isReal() && !Experiments.INSTANCE.enabled(Experiment.SimulateCameras)) {
            // Real robot, but without simulated cameras.
            return;
        }
        // publish the simulated tag sightings.
        m_tagSim.run();
        m_targetSim.run();
        // publish ground truth pose
        if (m_viz != null)
            m_viz.run();
    }
}
