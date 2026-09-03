package org.team100.lib.motor;

import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.music.Player;
import org.team100.lib.sensor.position.incremental.IncrementalEncoder;

/**
 * Methods pertain only to the output shaft, not the motion of the attached
 * mechanism. Accordingly, the units are always rotational, and there should be
 * no gear ratios in any implementation.
 */
public interface Motor extends Player, TotalCurrentLog.Reporter {

    ////////////////////////////////////////////////////////////
    ///
    /// ACTUATION
    ///

    /**
     * Some motors allow torque limiting through current limiting.
     * 
     * NOTE! Changing current limits can be a slow operation, so don't do this too
     * often.
     */
    void setTorqueLimit(double torqueNm);

    /**
     * Open-loop duty cycle control.
     * 
     * @param output in range [-1, 1]
     */
    void setDutyCycle(double output);

    /**
     * For tuning friction.
     * 
     * @param voltage volts
     */
    void setVoltage(double voltage);

    /**
     * For tuning inertia.
     * 
     * Be careful! This will go to full speed if you leave it on too long.
     * 
     * @param current amperes
     */
    void setCurrent(double current);

    /**
     * Velocity feedback with friction, velocity, acceleration, and holding torque.
     * 
     * There are two kinds of implementations, closed-loop and open-loop.
     * 
     * Closed-loop implementations use the velocity term as the controller target,
     * and also for feedforward based on back-EMF. Since back-EMF is intrinsic,
     * this feedforward is correct.
     * 
     * Previously, the closed-loop implementations used a multiplicative
     * feedforward for acceleration, which is the wrong place to do it, since
     * it properly depends on extrinsics (e.g. mechanism mass). This argument
     * is gone, and the extrinsics modeled "upstream."
     * 
     * The torque is used by closed-loop implementations using motor
     * intrisics, the torque constant and resistance (see getTorqueFFVolts()),
     * so this is correect as well.
     * 
     * Open-loop implementations just scale velocity to voltage or duty cycle
     * somehow, ignore the other terms, and don't expect to be very accurate.
     * 
     * @param velocityRad_S motor shaft speed, rad/s.
     * @param torqueNm      Nm, for gravity compensation or acceleration.
     */
    void setVelocity(
            double velocityRad_S,
            double torqueNm);

    /**
     * Position feedback with feedforward for friction, velocity, acceleration, and
     * holding torque.
     * 
     * Revolutions wind up; 0 != 2pi.
     * 
     * This is the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi
     * 
     * Should actuate immediately.
     * 
     * See setVelocity() for discussion of field uses in subclasses.
     * 
     * In the positional case, the open-loop implementations often don't
     * do anything at all.
     * 
     * Closed-loop implementations are closed on position, use velocity
     * for (correct, intrinsic) back-EMF feedforward, use torque for
     * (correct, intrinsic) R/kT feedforward.
     * 
     * Previously, incorrectly, there was a multiplier for kA.
     * 
     * @param positionRad   radians.
     * @param velocityRad_S rad/s.
     * @param torqueNm      Nm, for gravity compensation or acceleration.
     */
    void setUnwrappedPosition(
            double positionRad,
            double velocityRad_S,
            double torqueNm);

    /** This is not "hold position" this is "torque off". */
    void stop();

    /////////////////////////////////////////////////////////////
    ///
    /// MEASUREMENTS
    ///

    /**
     * "Unwrapped" angular motor shaft position, i.e. the measurement
     * domain continues beyond +/- pi. May be filtered.
     * 
     * Value should be updated in Robot.robotPeriodic().
     */
    double getUnwrappedPositionRad();

    /**
     * Motor shaft speed. May be filtered.
     * 
     * Value should be updated in Robot.robotPeriodic().
     */
    double getVelocityRad_S();

    /**
     * Motor shaft acceleration. May be filtered.
     * 
     * Value should be updated in Robot.robotPeriodic().
     */
    double getAccelerationRad_S2();

    /** Motor stator current in amps. */
    double getStatorCurrent();

    /**
     * This is the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi.
     */
    void setUnwrappedEncoderPositionRad(double positionRad);

    /////////////////////////////////////////////////////////
    ///
    /// MOTOR PARAMETER CONSTANTS
    ///

    /**
     * Motor resistance in ohms, used to calculate voltage from desired torque
     * current. This should be published by the manufacturer (divide stall current
     * by 12.0).
     * 
     * @return R value in ohms.
     */
    double R();

    /**
     * Motor torque constant, kT, in Nm per amp, used to calculate current from
     * desired torque. This should be published by the manufacturer (divide stall
     * torque by stall current).
     * 
     * @return kT value in Nm/amp.
     */
    double kT();

    /**
     * Back-EMF constant.
     * 
     * This is the voltage to maintain speed against the back-EMF of the motor.
     * 
     * V = kE * omega
     * 
     * so kE units are volt-sec/rad. This an intrinsic property of the motor.
     * https://en.wikipedia.org/wiki/Motor_constants#Motor_velocity_constant,_back_EMF_constant
     *
     * You can approximate kE using the motor free speed:
     * 
     * kE = 60 * 12 / (free speed * 2 * pi)
     * 
     * But you should really measure each motor. The measurement
     * is very easy: try a few voltages, measure the speed, find
     * the slope of the resulting line.
     * 
     * @return kE value in volt-sec/rad.
     */
    double kE();

    /**
     * Back-EMF voltage is simply proportional to speed:
     * 
     * V = kE * omega
     * 
     * To get the motor to actually go the requested speed, you should also add the
     * frictional offset.
     */
    default double backEMFVoltage(double motorRad_S) {
        return kE() * motorRad_S;
    }

    /**
     * Incremental voltage required to produce the given torque, used for
     * feedforward.
     */
    default double getTorqueFFVolts(double torqueNm) {
        double torqueFFAmps = torqueNm / kT();
        return torqueFFAmps * R();
    }

    /** Return encoder for this motor, if possible. */
    IncrementalEncoder encoder();

    /** Reset the cache. */
    void reset();

    /** For test cleanup. */
    void close();

    /** For logging */
    void periodic();

}
