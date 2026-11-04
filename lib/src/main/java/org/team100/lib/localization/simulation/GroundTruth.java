package org.team100.lib.localization.simulation;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.SideEffect;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.localization.map.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.targeting.simulation.SimulatedTargetWriter;
import org.team100.lib.util.TimeInterpolatableBuffer100;
import org.team100.lib.visualization.RobotPoseVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Uses noise-free odometry, alone, to compute the simulated robot ground-truth
 * pose.
 * 
 * To correctly simulate the influence of vision and gyro drift on robot
 * rotation, we need to track the "ground truth" of the robot separately from
 * the simulated measurements (which include drift).
 */
public class GroundTruth {
    private final SwerveDriveKinematics100 m_kinematics;
    private final SwerveModuleCollection m_modules;
    private final TimeInterpolatableBuffer100<StateSE2> m_estimates;
    private final SideEffect m_cache;
    private final SimulatedTagCornerDetector m_tagSim;
    private final SimulatedTargetWriter m_targetSim;

    // this only ever walks forward in time so we can just use
    // mutable fields instead of the full history with a key
    // query.
    private double previousTimestamp;
    private StateSE2 previousEstimate;
    private SwerveModulePositions previousMeasurement;

    public GroundTruth(
            LoggerFactory fieldLogger,
            LoggerFactory logger,
            SwerveKinodynamics kinodynamics,
            SwerveModuleCollection modules,
            AprilTagFieldLayoutWithCorrectOrientation layout) {
        LoggerFactory simLog = logger.name("Simulation");
        m_kinematics = kinodynamics.getKinematics();
        m_modules = modules;

        // History of ground-truth poses is based only on odometry.
        SwerveModulePositions initialMeasurement = m_modules.positions();
        StateSE2 initialEstimate = new StateSE2(Pose2d.kZero, VelocitySE2.ZERO);
        m_estimates = new TimeInterpolatableBuffer100<>(
                0.2, Takt.get(), initialEstimate);

        // Visualization of the simulated "ground truth" of the robot pose.
        new RobotPoseVisualization(fieldLogger, this::get, "ground truth");

        // the side-effect is updating the history and then publishing sightings
        m_cache = Cache.ofSideEffect(this::update);

        // Simulated camera uses the ground truth because the real cameras are not aware
        // of the pose estimate.
        m_tagSim = SimulatedTagCornerDetector.get(layout, m_estimates::get);
        m_targetSim = SimulatedTargetWriter.get(simLog, m_estimates::get);

        previousTimestamp = Takt.get();
        previousEstimate = initialEstimate;
        previousMeasurement = initialMeasurement;
    }

    void update() {
        if (RobotBase.isReal() && !Experiments.INSTANCE.enabled(Experiment.SimulateCameras)) {
            // Real robot, but without simulated cameras.
            return;
        }
        double timestamp = Takt.get();

        // Update the history with the new measurement
        SwerveModulePositions newMeasurement = m_modules.positions();
        StateSE2 newEstimate = estimate(
                previousTimestamp,
                previousEstimate,
                previousMeasurement,
                timestamp,
                newMeasurement);
        m_estimates.put(timestamp, newEstimate);

        // Publish the simulated tag and target sightings.
        m_tagSim.run();
        m_targetSim.run();

        previousTimestamp = timestamp;
        previousEstimate = newEstimate;
        previousMeasurement = newMeasurement;
    }

    /**
     * Update, if required, and return the state for the current time.
     */
    StateSE2 get() {
        m_cache.run();
        return m_estimates.get(Takt.get());
    }

    /**
     * Reset the pose with roughly infinite uncertainty, so the following update
     * will have a large effect.
     */
    public void resetPose(StateSE2 state) {
        if (RobotBase.isReal() && !Experiments.INSTANCE.enabled(Experiment.SimulateCameras)) {
            // Real robot, but without simulated cameras.
            return;
        }
        m_estimates.reset(Takt.get(), state);
    }

    /**
     * Use odometry only to derive an estimate.
     * 
     * @param previousTimestamp   for computing dt
     * @param previousEstimate    starting point for integration
     * @param previousMeasurement from the history
     * @param timestamp           current Takt time, seconds
     * @param newMeasurement      measurement at the timestamp
     * @return new estimate, never null
     */
    public StateSE2 estimate(
            double previousTimestamp,
            StateSE2 previousEstimate,
            SwerveModulePositions previousMeasurement,
            double timestamp,
            SwerveModulePositions newMeasurement) {
        double dt = timestamp - previousTimestamp;
        if (dt < 0.0001) {
            return previousEstimate;
        }
        SwerveModuleDeltas modulePositionDelta = SwerveModuleDeltas.modulePositionDelta(
                previousMeasurement, newMeasurement);
        Twist2d twist = m_kinematics.forward(modulePositionDelta);
        // The new pose is the twist applied to the old pose.
        Pose2d newPose = previousEstimate.pose().exp(twist);
        // Compute a new velocity using backward finite difference.
        VelocitySE2 velocity = VelocitySE2.velocity(previousEstimate.pose(), newPose, dt);
        return new StateSE2(newPose, velocity);
    }
}
