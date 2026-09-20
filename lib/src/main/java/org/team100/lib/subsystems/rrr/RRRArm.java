package org.team100.lib.subsystems.rrr;

import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.kinematics.rrr_se2.RRRFeasibility;
import org.team100.lib.kinematics.rrr_se2.RRRKinematicsPoE;
import org.team100.lib.subsystems.rn.PositionSubsystemRn;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

/**
 * NOTE: using the SE2 position API directly risks "flipping" in an uncontrolled
 * way. Test your SE2 paths in simulation before using them on a real mechanism.
 */
public interface RRRArm extends PositionSubsystemSE2, PositionSubsystemRn<N3> {

    RRRKinematicsPoE kinematics();

    RRRFeasibility feasibility();

    /**
     * Current measurement.
     */
    RRRConfig getConfig();

    /**
     * Current measured velocity.
     */
    RRRVelocity getVelocity();

    /**
     * Choose the feasible config closest to the current config.
     * 
     * @param p tool center point pose
     */
    RRRConfig config(Pose2d p);

    /**
     * Velocity for xdot, at config q.
     */
    RRRVelocity qdot(RRRConfig q, VelocitySE2 xdot);

    /**
     * Actuate, using dynamics to compute joint forces.
     */
    void set(RRRConfig q, RRRVelocity qdot, RRRAcceleration qddot);

    /**
     * Stop the mechanism. Depending on the brake mode of the motor, this may be a
     * "zero torque" condition, or a "braking" condition.
     */
    void stop();

    /**
     * Force all axis encoders to zero.
     */
    void setZero();
}
