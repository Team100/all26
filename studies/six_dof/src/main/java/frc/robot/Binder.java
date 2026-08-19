package frc.robot;

import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.se3.VelocitySE3;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;

public class Binder {

        private final Machinery m_machinery;

        public Binder(Machinery machinery) {
                m_machinery = machinery;
                Gamepad m_controller = new Gamepad(0);
                // button 1, "z" in sim
                // whileTrue(m_controller::getAButton, m_machinery.m_arm.warp0());
                // button 2, "x" in sim
                // whileTrue(m_controller::getAButton, m_machinery.m_arm.warp1());
                // button 3, "c" in sim
                // tool (x) pointing down
                // whileTrue(m_controller::getBButton,
                // m_machinery.m_arm.move(
                // new Pose3d(0.5, 0.25, 0.1, new Rotation3d(0, Math.PI / 2, 0))));
                // button 4, "v" in sim
                // Make a sequence of moves with various positions and orientations.
                MoveAndHold move1 = m_machinery.m_arm.move(
                                new Pose3d(0.5, 0.25, 0.1, new Rotation3d(0, 0, 0)));
                MoveAndHold move2 = m_machinery.m_arm.move(
                                new Pose3d(0.25, 0.25, 0.1, new Rotation3d(0, Math.PI / 2, 0)));
                MoveAndHold move3 = m_machinery.m_arm.move(
                                new Pose3d(0.25, -0.25, 0.1, new Rotation3d(0, 0, 0)));
                MoveAndHold move4 = m_machinery.m_arm.move(
                                new Pose3d(0.5, -0.25, 0.1, new Rotation3d(0, Math.PI / 2, 0)));
                whileTrue(m_controller::getSouthFaceButton,
                                move1.until(move1::isDone)
                                                .andThen(move2.until(move2::isDone))
                                                .andThen(move3.until(move3::isDone))
                                                .andThen(move4.until(move4::isDone)));

                // dx chosen to be collinear with end-effector
                MoveAndHold move1s = m_machinery.m_arm.moveSplined(
                                new VelocitySE3(0, 0, 0.5, 0, 0, 0),
                                new Pose3d(0.5, 0.25, 0.1, new Rotation3d(0, 0, 0)),
                                new VelocitySE3(0.5, 0, 0, 0, 0, 0));
                MoveAndHold move2s = m_machinery.m_arm.moveSplined(
                                new VelocitySE3(-0.5, 0, 0, 0, 0, 0),
                                new Pose3d(0.25, 0.25, 0.1, new Rotation3d(0, Math.PI / 2, 0)),
                                new VelocitySE3(0, 0, -0.5, 0, 0, 0));
                MoveAndHold move3s = m_machinery.m_arm.moveSplined(
                                new VelocitySE3(0, 0, 0.5, 0, 0, 0),
                                new Pose3d(0.25, -0.25, 0.1, new Rotation3d(0, 0, 0)),
                                new VelocitySE3(0.5, 0, 0, 0, 0, 0));
                MoveAndHold move4s = m_machinery.m_arm.moveSplined(
                                new VelocitySE3(-0.5, 0, 0, 0, 0, 0),
                                new Pose3d(0.5, -0.25, 0.1, new Rotation3d(0, Math.PI / 2, 0)),
                                new VelocitySE3(0, 0, -0.5, 0, 0, 0));
                whileTrue(m_controller::getEastFaceButton,
                                move1s.until(move1s::isDone)
                                                .andThen(Commands.waitSeconds(0.5))
                                                .andThen(move2s.until(move2s::isDone))
                                                .andThen(Commands.waitSeconds(0.5))
                                                .andThen(move3s.until(move3s::isDone))
                                                .andThen(Commands.waitSeconds(0.5))
                                                .andThen(move4s.until(move4s::isDone)));

                // circle a target
                MoveAndHold s1 = m_machinery.m_arm.move(
                                new Pose3d(0.4, 0, 0.3, new Rotation3d(0, 0, Math.PI / 4)));
                MoveAndHold s2 = m_machinery.m_arm.move(
                                new Pose3d(0.4, 0, 0.3, new Rotation3d(0, Math.PI / 4, 0)));
                MoveAndHold s3 = m_machinery.m_arm.move(
                                new Pose3d(0.4, 0, 0.3, new Rotation3d(0, 0, -Math.PI / 4)));
                MoveAndHold s4 = m_machinery.m_arm.move(
                                new Pose3d(0.4, 0, 0.3, new Rotation3d(0, -Math.PI / 4, 0)));
                whileTrue(m_controller::getWestFaceButton,
                                s1.until(s1::isDone)
                                                .andThen(s2.until(s2::isDone))
                                                .andThen(s3.until(s3::isDone))
                                                .andThen(s4.until(s4::isDone)));
        }

        public void close() {
        }
}
