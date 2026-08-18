package org.team100.lib.subsystems.swerve;

import java.util.List;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.DriverSkill;
import org.team100.lib.dynamics.swerve.SwerveEffort;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.EnumLogger;
import org.team100.lib.logging.LoggerFactory.ModelSE2Logger;
import org.team100.lib.logging.LoggerFactory.VelocityControlSE2Logger;
import org.team100.lib.music.Music;
import org.team100.lib.music.Player;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.visualization.VizUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase implements VelocitySubsystemSE2, Music {
    // DEBUG produces a LOT of output. Only enable it while you're looking.
    private static final boolean DEBUG = false;
    private final FreshSwerveEstimate m_estimate;
    private final OdometryUpdater m_odometryUpdater;
    private final SwerveLocal m_swerveLocal;

    // CACHE
    private final ObjectCache<ModelSE2> m_stateCache;

    // LOGGERS
    private final ModelSE2Logger m_log_state;
    private final DoubleArrayLogger m_log_pose_array;
    private final EnumLogger m_log_skill;
    private final VelocityControlSE2Logger m_log_input;
    private final DoubleLogger m_log_rotation_evolution;

    private final List<Player> m_players;

    public SwerveDriveSubsystem(
            LoggerFactory parent,
            OdometryUpdater odometryUpdater,
            FreshSwerveEstimate estimate,
            SwerveLocal swerveLocal) {
        LoggerFactory log = parent.type(this);
        m_estimate = estimate;
        m_odometryUpdater = odometryUpdater;
        m_swerveLocal = swerveLocal;
        m_stateCache = Cache.of(this::update);
        m_log_state = log.modelSE2Logger(Level.COMP, "state");
        m_log_pose_array = log.doubleArrayLogger(Level.COMP, "pose array");
        m_log_skill = log.enumLogger(Level.TRACE, "skill level");
        m_log_input = log.velocityControlSE2Logger(Level.TRACE, "drive input");
        m_log_rotation_evolution = log.doubleLogger(Level.TRACE, "rotation evolution");
        m_players = m_swerveLocal.players();
        stop();
    }

    ////////////////
    //
    // ACTUATORS
    //

    /**
     * Drive using field coordinates
     * 
     * @param nextV for the next timestep
     */
    @Override
    public void set(VelocityControlSE2 nextV) {
        // Actuation is constant for the whole control period, which means
        // that to calculate robot-relative speed from field-relative speed,
        // we need to use the robot rotation *at the future time*.
        ModelSE2 currentState = getState();
        // Note this may add a bit of noise.
        ModelSE2 nextState = currentState.evolve(TimedRobot100.LOOP_PERIOD_S);
        Rotation2d nextTheta = nextState.rotation();
        // is there noise here?
        m_log_rotation_evolution.log(
                () -> nextTheta.minus(currentState.rotation()).getRadians());
        ChassisSpeeds nextSpeed = SwerveKinodynamics.toInstantaneousChassisSpeeds(
                nextV.velocity(), nextTheta);
        ChassisAcceleration nextAccel = ChassisAcceleration.fromFieldRelative(
                nextV.acceleration(), nextTheta);
        m_swerveLocal.setChassisSpeeds(nextSpeed, nextAccel);
        m_log_input.log(() -> nextV);
    }

    /**
     * Drive in robot-relative coordinates.
     */
    public void setChassisSpeeds(ChassisSpeeds speeds, ChassisAcceleration accel) {
        m_swerveLocal.setChassisSpeeds(speeds, accel);
    }

    /**
     * For testing only.
     */
    public void setRawModuleStates(SwerveModuleStates states, SwerveEffort effort) {
        m_swerveLocal.setRawModuleStates(states, effort);
    }

    @Override
    public void stop() {
        m_swerveLocal.stop();
    }

    /**
     * Empty the pose history, reset the servos, add the given pose, and flush the
     * cache.
     */
    public void resetPose(Pose2d robotPose, IsotropicNoiseSE2 noise) {
        if (DEBUG)
            System.out.println("WARNING: Make sure resetting the swerve module collection doesn't break anything");
        m_swerveLocal.reset();
        m_odometryUpdater.reset(robotPose, noise);
        m_stateCache.reset();
    }

    /** Makes an X, stopped. */
    public void defense() {
        m_swerveLocal.defense();
    }

    ///////////////////////////////////////////////////////////////
    //
    // Observers
    //

    /**
     * Cached.
     * 
     * SwerveState representing the drivetrain's field-relative pose, velocity, and
     * acceleration.
     */
    @Override
    public ModelSE2 getState() {
        return m_stateCache.get();
    }

    ///////////////////////////////////////////////////////////////

    /**
     * Periodic() should not do actuation. Let commands do that.
     */
    @Override
    public void periodic() {
        if (DEBUG)
            System.out.println("drive periodic");
        m_swerveLocal.periodic();
        m_log_state.log(this::getState);
        m_log_pose_array.log(this::poseArray);
        m_log_skill.log(() -> DriverSkill.level());
    }

    private double[] poseArray() {
        return VizUtil.poseToArray(getPose());
    }

    public void close() {
        m_swerveLocal.close();
    }

    /** Return cached pose. */
    public Pose2d getPose() {
        return m_stateCache.get().pose();
    }

    /** Return cached velocity. */
    public VelocitySE2 getVelocity() {
        return m_stateCache.get().velocity();
    }

    /** Return cached speeds. */
    public ChassisSpeeds getChassisSpeeds() {
        return m_stateCache.get().chassisSpeeds();
    }

    @Override
    public List<Player> players() {
        return m_players;
    }

    ///////////////////////////////////////////////////////////////
    //
    // Commands
    //

    /** Stop and then end -- this is for compositions where doing nothing is OK */
    public Command stopOnce() {
        return runOnce(this::stop).withName("Drive Stop Once");
    }

    /**
     * Use raw mode to set modules driving ahead.
     * Never ends.
     */
    public Command aheadSlow() {
        return run(() -> setRawModuleStates(
                SwerveModuleStates.aheadSlow, SwerveEffort.ZERO))
                .withName("Drive Ahead");

    }

    /**
     * Use robot-relative mode to set modules driving to the right.
     * Never ends.
     */
    public Command rightwardSlow() {
        return run(() -> setChassisSpeeds(
                new ChassisSpeeds(0, -1.0, 0), ChassisAcceleration.ZERO))
                .withName("Drive Right Slow");
    }

    /**
     * Use robot-relative mode to spin to the left.
     * Never ends.
     */
    public Command spinLeft() {
        return run(() -> setChassisSpeeds(
                new ChassisSpeeds(0, 0, 1.0), ChassisAcceleration.ZERO))
                .withName("Drive Spin Left");
    }

    /**
     * This can conflict with the apriltag input and cause the robot to lose its
     * mind. Do not use it without understanding it and testing it in a safe
     * environment.
     */
    public Command resetPoseCommand(Pose2d pose) {
        return runOnce(() -> resetPose(pose, IsotropicNoiseSE2.high()));
    }

    /**
     * This can conflict with the apriltag input and cause the robot to lose its
     * mind. Do not use it without understanding it and testing it in a safe
     * environment.
     */
    public Command setRotationCommand(Rotation2d rotation) {
        return runOnce(() -> resetPose(
                new Pose2d(getPose().getTranslation(), rotation),
                IsotropicNoiseSE2.high()));
    }

    @Override
    public Command play(double freq) {
        return run(() -> {
            m_swerveLocal.play(freq);
        });
    }

    public Command defend() {
        return run(this::defense);
    }

    /////////////////////////////////////////////////////////////////

    /**
     * Compute the current state. This is a fairly heavyweight thing to do, so it
     * should be cached (thus refreshed once per cycle).
     */
    private ModelSE2 update() {
        double now = Takt.get();
        SwerveModulePositions positions = m_swerveLocal.positions();
        ModelSE2 swerveModel = m_estimate.apply(now);
        if (DEBUG) {
            System.out.printf("update() positions %s estimated pose: %s\n", positions, swerveModel);
        }
        return swerveModel;
    }

}
