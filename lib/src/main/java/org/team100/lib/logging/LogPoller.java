package org.team100.lib.logging;

import java.util.ArrayList;
import java.util.List;

/**
 * Polls for periodic logs. This avoids wiring up the "periodic" tree.
 */
public class LogPoller {
    private static final List<Runnable> targets = new ArrayList<>();

    public static void register(Runnable target) {
        targets.add(target);
    }

    /** Run the registrant logs. */
    public static void log() {
        targets.forEach(Runnable::run);
    }

}
