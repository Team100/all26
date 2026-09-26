package org.team100.lib.subsystems.swerve.module;

import java.util.List;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Identity;
import org.team100.lib.dynamics.swerve.SwerveEffort;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.music.Player;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;

/**
 * Represents the modules in the drivetrain.
 * Do not put logic here; this is just for bundling the modules together.
 * 
 * HOW TO CALIBRATE THE STEERING
 * 
 * 1. align the bevels to the right
 * 2. find the "position (turns)" in glass
 * 3. copy the value there into the offset argument
 * 4. deploy and check that "position (turns-offset)" is zero.
 */
public class SwerveModuleCollection implements Player {
    private static final boolean DEBUG = false;
    private final SwerveModule100 m_frontLeft;
    private final SwerveModule100 m_frontRight;
    private final SwerveModule100 m_rearLeft;
    private final SwerveModule100 m_rearRight;

    private final List<Player> m_players;

    SwerveModuleCollection(
            SwerveModule100 frontLeft,
            SwerveModule100 frontRight,
            SwerveModule100 rearLeft,
            SwerveModule100 rearRight) {
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
        m_players = List.of(
                m_frontLeft.players(),
                m_frontRight.players(),
                m_rearLeft.players(),
                m_rearRight.players())
                .stream().flatMap(List::stream).toList();
    }

    @Override
    public List<Player> players() {
        return m_players;
    }

    /**
     * Creates collections according to Identity.
     */
    public static SwerveModuleCollection get(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit driveLimit,
            CurrentLimit steerLimit) {
        LoggerFactory log = parent.name("Swerve Modules");
        switch (Identity.instance) {
            case SQUAREBOT:
                return new SwerveModulesSquare(
                        log, currentLog, driveLimit, steerLimit);
            case SWERVE_ONE:
                return new SwerveModulesPractice(
                        log, currentLog, driveLimit, steerLimit);
            case BETA_BOT, LAUNDRY_BOT:
                return new SwerveModulesComp9999(
                        log, currentLog, driveLimit, steerLimit);
            case BLANK:
            default:
                return new SwerveModulesSim(log);
        }
    }

    //////////////////////////////////////////////////
    //
    // Actuators
    //

    /**
     * Optimizes.
     * 
     * Works fine with empty angles.
     * 
     * @param nextStates for now+dt. Avoid noise here.
     * @param effort     force
     */
    public void setDesiredStates(
            SwerveModuleStates nextStates, SwerveEffort effort) {
        if (DEBUG) {
            System.out.printf("setDesiredStates() %s\n", nextStates);
        }
        m_frontLeft.setDesiredState(nextStates.frontLeft(), effort.fl());
        m_frontRight.setDesiredState(nextStates.frontRight(), effort.fr());
        m_rearLeft.setDesiredState(nextStates.rearLeft(), effort.rl());
        m_rearRight.setDesiredState(nextStates.rearRight(), effort.rr());
    }

    /**
     * Does not optimize.
     * 
     * This "raw" mode is just for testing.
     * 
     * Works fine with empty angles.
     * 
     * @param swerveModuleStates. Avoid noise in these inputs.
     * @param effort              Forces.
     */
    public void setRawDesiredStates(
            SwerveModuleStates swerveModuleStates, SwerveEffort effort) {
        m_frontLeft.setRawDesiredState(swerveModuleStates.frontLeft(), effort.fl());
        m_frontRight.setRawDesiredState(swerveModuleStates.frontRight(), effort.fr());
        m_rearLeft.setRawDesiredState(swerveModuleStates.rearLeft(), effort.rl());
        m_rearRight.setRawDesiredState(swerveModuleStates.rearRight(), effort.rr());
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }

    //////////////////////////////////////////////////////
    //
    // Observers
    //

    /** Uses Cache so the positions are fresh and coherent. */
    public SwerveModulePositions positions() {
        return new SwerveModulePositions(
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition());
    }

    /** FOR TEST ONLY */
    public SwerveModuleStates states() {
        return new SwerveModuleStates(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
    }

    ////////////////////////////////////////////

    public void close() {
        m_frontLeft.close();
        m_frontRight.close();
        m_rearLeft.close();
        m_rearRight.close();
    }

    public SwerveModule100[] modules() {
        return new SwerveModule100[] {
                m_frontLeft,
                m_frontRight,
                m_rearLeft,
                m_rearRight };
    }

    @Override
    public void play(double freq) {
        m_frontLeft.play(freq);
        m_frontRight.play(freq);
        m_rearLeft.play(freq);
        m_rearRight.play(freq);
    }
}
