package org.team100.lib.subsystems.swerve;

import java.util.List;

import org.team100.lib.coherence.Takt;
import org.team100.lib.dynamics.swerve.SwerveEffort;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.localization.StateEstimator;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.StateSE2Logger;
import org.team100.lib.logging.LoggerFactory.VelocityControlSE2Logger;
import org.team100.lib.music.Music;
import org.team100.lib.music.Player;
import org.team100.lib.state.StateSE2;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The swerve subsystem is a thin subsystem wrapper that provides pose
 * estimation and local actuation.
 */
public class SwerveDriveSubsystem extends SubsystemBase implements VelocitySubsystemSE2, Music {
    private final StateEstimator m_estimate;
    private final SwerveLocal m_swerveLocal;
    private final StateSE2Logger m_log_state;
    private final VelocityControlSE2Logger m_log_input;
    private final DoubleLogger m_log_rotation_evolution;
    private final List<Player> m_players;

    public SwerveDriveSubsystem(
            LoggerFactory parent,
            StateEstimator estimate,
            SwerveLocal swerveLocal) {
        LoggerFactory log = parent.type(this);
        m_estimate = estimate;
        m_swerveLocal = swerveLocal;
        m_log_state = log.StateSE2Logger(Level.COMP, "state");
        m_log_input = log.velocityControlSE2Logger(Level.TRACE, "drive input");
        m_log_rotation_evolution = log.doubleLogger(Level.TRACE, "rotation evolution");
        m_players = m_swerveLocal.players();
        LogPoller.register(this::log);
        stop();
    }

    ////////////////
    //
    // ACTUATORS
    //

    /**
     * Drive using field coordinates.
     * 
     * @param nextV For the next timestep. Avoid noise here.
     */
    @Override
    public void set(VelocityControlSE2 nextV) {
        // Actuation is constant for the whole control period, which means
        // that to calculate robot-relative speed from field-relative speed,
        // we need to use the robot rotation *at the future time*.
        StateSE2 currentState = getState();
        // Note this may add a bit of noise.
        StateSE2 nextState = currentState.evolve(TimedRobot100.LOOP_PERIOD_S);
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
     * Drive using robot-relative coordinates.
     */
    public void setChassisSpeeds(ChassisSpeeds speeds, ChassisAcceleration accel) {
        m_swerveLocal.setChassisSpeeds(speeds, accel);
    }

    /**
     * Set module states directly. For testing only.
     */
    public void setRawModuleStates(SwerveModuleStates states, SwerveEffort effort) {
        m_swerveLocal.setRawModuleStates(states, effort);
    }

    @Override
    public void stop() {
        m_swerveLocal.stop();
    }

    /** Empty the pose history, add the given pose, and flush the cache. */
    public void resetPose(Pose2d robotPose, IsotropicNoiseSE2 noise) {
        m_estimate.reset(robotPose, noise);
    }

    ///////////////////////////////////////////////////////////////
    //
    // Observers
    //

    /** Sample the state at the current time. */
    @Override
    public StateSE2 getState() {
        return getState(Takt.get());
    }

    /** Sample the past state at the specified time. */
    public StateSE2 getState(double timeSec) {
        return m_estimate.get(timeSec);
    }

    /** Tags outside this radius are ignored. */
    public void setHeedRadiusM(double heedRadiusM) {
        m_estimate.setHeedRadiusM(heedRadiusM);
    }

    ///////////////////////////////////////////////////////////////
    //
    // Commands
    //

    /** Stop and then end -- this is for compositions where doing nothing is OK */
    public Command stopOnce() {
        return runOnce(this::stop)
                .withName("Drive Stop");
    }

    /** Drive to the robot's front, endlessly. */
    public Command aheadSlow() {
        return run(() -> setRawModuleStates(
                SwerveModuleStates.aheadSlow, SwerveEffort.ZERO))
                .withName("Drive Ahead");
    }

    /** Drive to the robot's right, endlessly. */
    public Command rightwardSlow() {
        return run(() -> setChassisSpeeds(
                new ChassisSpeeds(0, -1.0, 0), ChassisAcceleration.ZERO))
                .withName("Drive Right");
    }

    /** Spin counterclockwise, endlessly. */
    public Command spinLeft() {
        return run(() -> setChassisSpeeds(
                new ChassisSpeeds(0, 0, 1.0), ChassisAcceleration.ZERO))
                .withName("Drive Spin");
    }

    /** Hold the wheels in an "X" pattern, endlessly. */
    public Command defend() {
        return run(m_swerveLocal::defense)
                .withName("Drive Defend");
    }

    @Override
    public Command play(double freq) {
        return run(() -> {
            m_swerveLocal.play(freq);
        });
    }

    @Override
    public List<Player> players() {
        return m_players;
    }

    private void log() {
        m_log_state.log(this::getState);
    }

    public void close() {
        m_swerveLocal.close();
    }

}
