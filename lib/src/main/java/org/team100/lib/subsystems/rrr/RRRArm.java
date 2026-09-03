package org.team100.lib.subsystems.rrr;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.subsystems.rn.PositionSubsystemRn;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;
import org.wpilib.command2.Command;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.numbers.N3;

public interface RRRArm extends PositionSubsystemSE2, PositionSubsystemRn<N3> {

    RRRConfig getConfig();

    RRRConfig getConfigWithinLimits();

    RRRConfig config(Pose2d p);

    RRRVelocity qdot(RRRConfig q, VelocitySE2 xdot);

    Pose2d pose();

    void set(RRRConfig q, RRRVelocity qdot, RRRAcceleration qddot);

    void stop();

    double l1();

    double l2();

    double l3();

    MoveAndHold moveProfiled(ProfileR1 profile, Pose2d goal);

    MoveAndHold moveTrajSE2(Pose2d goal, double speed);

    MoveAndHold moveSplined(VelocitySE2 x0dot, Pose2d x1, VelocitySE2 x1dot);

    Command moveManually(Gamepad controller);

}
