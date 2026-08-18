package org.team100.frc2026.auton;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import java.util.List;
import java.util.function.Function;

import org.team100.frc2026.robot.Machinery;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.trajectory.se2.TrajectorySE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An auton that does nothing at all. */
public class JustShoot implements AnnotatedCommand {
    private final Machinery m_machinery;

    public JustShoot(
            Machinery machinery) {
        m_machinery = machinery;
    }

    @Override
    public String name() {
        return "Just Shoot";
    }

    @Override
    public Command command() {
        return parallel(
                m_machinery.m_shooter.auto()).withName("Shoot").withTimeout(5);
    }

    @Override
    public Pose2d start() {
        return StartingPositions.CENTER;
    }

    @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        return List.of();
    }
}
