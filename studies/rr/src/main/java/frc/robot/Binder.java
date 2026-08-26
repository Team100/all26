package frc.robot;

import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.WPITrapezoidProfileR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

public class Binder {
    private static final boolean BLARG = false;

    private final Machinery m_machinery;

    public Binder(Machinery machinery) {
        m_machinery = machinery;
        XboxController m_controller = new XboxController(0);
        // "A" is button 1, "z" in sim
        // whileTrue(m_controller::getAButton, m_machinery.m_arm.warp0());
        // "B" is button 2, "x" in sim
        // whileTrue(m_controller::getBButton, m_machinery.m_arm.warp1());
        // "X" is button 3, "c" in sim
        // tool (x) pointing down
        // whileTrue(m_controller::getXButton, // "c"
        // m_machinery.m_arm.moveProfiled(
        // new Pose2d(0.5, 0.25, new Rotation2d(0))));

        // "Y" is button 4, "v" in sim
        ProfileR1 profile = new WPITrapezoidProfileR1(3, 6);

        // profiles in joint space make kinda circular paths in workspace
        MoveAndHold move1 = m_machinery.m_arm.moveProfiled(profile,
                new Translation2d(0.2, 0.25));
        MoveAndHold move2 = m_machinery.m_arm.moveProfiled(profile,
                new Translation2d(0.4, 0.25));
        MoveAndHold move3 = m_machinery.m_arm.moveProfiled(profile,
                new Translation2d(0.4, -0.25));
        MoveAndHold move4 = m_machinery.m_arm.moveProfiled(profile,
                new Translation2d(0.2, -0.25));
        whileTrue(m_controller::getAButton,
                move1.until(move1::isDone)
                        .andThen(move2.until(move2::isDone))
                        .andThen(move3.until(move3::isDone))
                        .andThen(move4.until(move4::isDone)));

        // spline ends are kinda straighter
        // note approach directions.
        MoveAndHold move1s = m_machinery.m_arm.moveSplined(
                new VelocityR2(0.5, 0),
                new Translation2d(0.2, 0.25),
                new VelocityR2(-0.5, 0));
        MoveAndHold move2s = m_machinery.m_arm.moveSplined(
                new VelocityR2(0, -0.5),
                new Translation2d(0.4, 0.25),
                new VelocityR2(0, 0.5));
        MoveAndHold move3s = m_machinery.m_arm.moveSplined(
                new VelocityR2(-0.5, 0),
                new Translation2d(0.4, -0.25),
                new VelocityR2(0.5, 0));
        MoveAndHold move4s = m_machinery.m_arm.moveSplined(
                new VelocityR2(0, 0.5),
                new Translation2d(0.2, -0.25),
                new VelocityR2(0, -0.5));
        whileTrue(m_controller::getBButton,
                move1s.until(move1s::isDone)
                        .andThen(move2s.until(move2s::isDone))
                        .andThen(move3s.until(move3s::isDone))
                        .andThen(move4s.until(move4s::isDone)));

        // trajectories in workspace make straight lines in workspace
        MoveAndHold move1t = m_machinery.m_arm.moveTrajSE2(
                new Pose2d(0.2, 0.25, new Rotation2d(0)), 1);
        MoveAndHold move2t = m_machinery.m_arm.moveTrajSE2(
                new Pose2d(0.4, 0.25, new Rotation2d(0)), 1);
        MoveAndHold move3t = m_machinery.m_arm.moveTrajSE2(
                new Pose2d(0.4, -0.25, new Rotation2d(0)), 1);
        MoveAndHold move4t = m_machinery.m_arm.moveTrajSE2(
                new Pose2d(0.2, -0.25, new Rotation2d(0)), 1);
        whileTrue(m_controller::getXButton,
                move1t.until(move1t::isDone)
                        .andThen(move2t.until(move2t::isDone))
                        .andThen(move3t.until(move3t::isDone))
                        .andThen(move4t.until(move4t::isDone)));

        // this shows how the inverse kinematics selects the right
        // way to go, and also how joint profiles don't care about
        // covering any R2 ground
        MoveAndHold s1 = m_machinery.m_arm.moveProfiled(profile,
                new Translation2d(0.4, 0));
        MoveAndHold s2 = m_machinery.m_arm.moveProfiled(profile,
                new Translation2d(0.4, 0));
        MoveAndHold s3 = m_machinery.m_arm.moveProfiled(profile,
                new Translation2d(0.4, 0));
        MoveAndHold s4 = m_machinery.m_arm.moveProfiled(profile,
                new Translation2d(0.4, 0));
        MoveAndHold s5 = m_machinery.m_arm.moveProfiled(profile,
                new Translation2d(0.4, 0));
        MoveAndHold s6 = m_machinery.m_arm.moveProfiled(profile,
                new Translation2d(0.4, 0));
        MoveAndHold s7 = m_machinery.m_arm.moveProfiled(profile,
                new Translation2d(0.4, 0));
        MoveAndHold s8 = m_machinery.m_arm.moveProfiled(profile,
                new Translation2d(0.4, 0));
        whileTrue(m_controller::getYButton, // "c"
                s1.until(s1::isDone)
                        .andThen(s2.until(s2::isDone))
                        .andThen(s3.until(s3::isDone))
                        .andThen(s4.until(s4::isDone))
                        .andThen(s5.until(s5::isDone))
                        .andThen(s6.until(s6::isDone))
                        .andThen(s7.until(s7::isDone))
                        .andThen(s8.until(s8::isDone)));

        if (BLARG) {
            // this uses slightly-offset points so that the spline-maker
            // doesn't freak out.
            // This illustrates that the trajectory planner is unaware of the
            // arm configuration, and drives it into its limits. The simulation
            // just "warps" to the other side of the limit; the real mechanism
            // would not.
            double v = 0.01; // velocity is x/y only
            MoveAndHold st1 = m_machinery.m_arm.moveTrajSE2(
                    new Pose2d(0.39, 0, new Rotation2d(0)), v);
            MoveAndHold st2 = m_machinery.m_arm.moveTrajSE2(
                    new Pose2d(0.39, -0.01, new Rotation2d(Math.PI / 4)), v);
            MoveAndHold st3 = m_machinery.m_arm.moveTrajSE2(
                    new Pose2d(0.4, -0.01, new Rotation2d(Math.PI / 2)), v);
            MoveAndHold st4 = m_machinery.m_arm.moveTrajSE2(
                    new Pose2d(0.41, -0.01, new Rotation2d(3 * Math.PI / 4)), v);
            MoveAndHold st5 = m_machinery.m_arm.moveTrajSE2(
                    new Pose2d(0.41, 0, new Rotation2d(Math.PI)), v);
            MoveAndHold st6 = m_machinery.m_arm.moveTrajSE2(
                    new Pose2d(0.41, 0.01, new Rotation2d(-3 * Math.PI / 4)), v);
            MoveAndHold st7 = m_machinery.m_arm.moveTrajSE2(
                    new Pose2d(0.4, 0.01, new Rotation2d(-Math.PI / 2)), v);
            MoveAndHold st8 = m_machinery.m_arm.moveTrajSE2(
                    new Pose2d(0.39, 0.01, new Rotation2d(-Math.PI / 4)), v);
            whileTrue(m_controller::getYButton, // "v"
                    st1.until(st1::isDone)
                            .andThen(st2.until(st2::isDone))
                            .andThen(st3.until(st3::isDone))
                            .andThen(st4.until(st4::isDone))
                            .andThen(st5.until(st5::isDone))
                            .andThen(st6.until(st6::isDone))
                            .andThen(st7.until(st7::isDone))
                            .andThen(st8.until(st8::isDone)));
        }
    }

    public void close() {
    }
}
