package org.team100.lib.subsystems.swerve;

import java.util.List;
import java.util.Optional;

import org.team100.lib.dynamics.swerve.SwerveEffort;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.logging.LoggerFactory.SwerveModulePositionsLogger;
import org.team100.lib.music.Player;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleState100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The swerve drive in local, or robot, reference frame. This class knows
 * nothing about the outside world, it just accepts chassis speeds.
 * 
 * Most methods in this class should be package-private, they're only used by
 * SwerveDriveSubsystem, and by tests.
 */
public class SwerveLocal implements Player {
    private static final SwerveModuleStates statesX = new SwerveModuleStates(
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(-1 * Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(3 * Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(-3 * Math.PI / 4))));

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final SwerveModuleCollection m_modules;

    private final SwerveModulePositionsLogger m_logPositions;
    private final ChassisSpeedsLogger m_log_chassis_speed;

    private final List<Player> m_players;

    public SwerveLocal(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            SwerveModuleCollection modules) {
        LoggerFactory log = parent.type(this);
        m_log_chassis_speed = log.chassisSpeedsLogger(Level.TRACE, "chassis speed");
        m_logPositions = log.swerveModulePositionsLogger(Level.TRACE, "positions");
        m_swerveKinodynamics = swerveKinodynamics;
        m_modules = modules;
        m_players = m_modules.players();
    }

    @Override
    public void play(double freq) {
        m_modules.play(freq);
    }

    @Override
    public List<Player> players() {
        return m_players;
    }

    //////////////////////////////////////////////////////////
    //
    // Actuators. These are mutually exclusive within an iteration.
    //

    /**
     * Discretizes the speeds, calculates the inverse kinematic module states, and
     * sets the module states.
     * 
     * @param nextSpeed for the next timestep.
     */
    void setChassisSpeeds(ChassisSpeeds nextSpeed, ChassisAcceleration a) {
        SwerveModuleStates states = m_swerveKinodynamics.toSwerveModuleStates(nextSpeed);
        SwerveEffort effort = m_swerveKinodynamics.effort(states, a);
        m_modules.setDesiredStates(states, effort);
        m_log_chassis_speed.log(() -> nextSpeed);
    }

    void stop() {
        m_modules.stop();
    }

    /**
     * Set the module states directly. This is just for testing.
     */
    void setRawModuleStates(
            SwerveModuleStates targetModuleStates, SwerveEffort effort) {
        m_modules.setRawDesiredStates(targetModuleStates, effort);
    }

    /**
     * Sets the wheels to make an "X" pattern.
     */
    void defense() {
        setRawModuleStates(statesX, SwerveEffort.ZERO);
    }

    ////////////////////////////////////////////////////////////////////
    //
    // Observers
    //

    /** Uses Cache so the position is fresh and coherent. */
    SwerveModulePositions positions() {
        return m_modules.positions();
    }

    ///////////////////////////////////////////

    void close() {
        m_modules.close();
    }

    /** Set turning setpoint to measurement, zero drive encoder. */
    void reset() {
        m_modules.reset();
    }

    /** Updates visualization. */
    void periodic() {
        m_logPositions.log(this::positions);
        m_modules.periodic();
    }
}
