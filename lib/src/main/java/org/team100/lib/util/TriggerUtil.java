package org.team100.lib.util;

import java.util.function.BooleanSupplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.button.Trigger;

public class TriggerUtil {
    public static Trigger whileTrue(BooleanSupplier condition, Command command) {
        return new Trigger(condition).whileTrue(command);
    }

    public static Trigger onTrue(BooleanSupplier condition, Command command) {
        return new Trigger(condition).onTrue(command);
    }
}
