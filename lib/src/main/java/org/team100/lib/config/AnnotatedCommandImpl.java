package org.team100.lib.config;

import java.util.List;
import java.util.function.Function;

import org.team100.lib.trajectory.se2.TrajectorySE2;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.Command;

/** Legacy implementation */
public class AnnotatedCommandImpl implements AnnotatedCommand {
    private final String name;
    private final Command command;
    private final Alliance alliance;
    private final Pose2d start;

    public AnnotatedCommandImpl(
            String name,
            Command command,
            Alliance alliance,
            Pose2d start) {
        this.name = name;
        this.command = command;
        this.alliance = alliance;
        this.start = start;
    }

    public AnnotatedCommandImpl(String name, Command command) {
        this(name, command, null, null);
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public Command command() {
        return command;
    }

    @Override
    public Alliance alliance() {
        return alliance;
    }

    @Override
    public Pose2d start() {
        return start;
    }

    @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        return List.of();
    }
}
