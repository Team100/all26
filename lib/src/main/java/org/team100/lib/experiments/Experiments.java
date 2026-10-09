package org.team100.lib.experiments;

import java.util.EnumMap;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controls Experiment enablement.
 * 
 * There are three methods of enablement:
 * 
 * -- defaults: enabled for all robots
 * -- override: using a Sendable Chooser in a dashboard, e.g. glass.
 * -- test override: to force a config for unit tests.
 * 
 * To get the experiment selectors to appear in glass, the usual procedure is:
 * 
 * Experiments.INSTANCE.show();
 */
public class Experiments {
    public static final Experiments INSTANCE = new Experiments();

    /** These experiments are enabled by default. */
    private final Set<Experiment> m_defaults = Set.of(
            Experiment.IncludeFrictionFeedForward,
            Experiment.IncludeVelocityFeedForward,
            Experiment.IncludeTorqueFeedForward);

    /** Key = experiment, value = enabled. */
    private final Map<Experiment, Boolean> m_enabled;

    private Experiments() {
        m_enabled = new EnumMap<>(Experiment.class);
        System.out.println("===============================================================");
        System.out.println("== EXPERIMENTS");
        for (Experiment e : Experiment.values()) {
            SendableChooser<Boolean> widget = ExperimentChooser.get(e.name());
            System.out.printf("== %s (%s): ", e.name(), e.description);
            if (m_defaults.contains(e)) {
                widget.setDefaultOption(on(e), true);
                widget.addOption(off(e), false);
                m_enabled.put(e, true);
                System.out.println("ON");
            } else {
                widget.addOption(on(e), true);
                widget.setDefaultOption(off(e), false);
                m_enabled.put(e, false);
                System.out.println("OFF");
            }
            widget.onChange(selected -> m_enabled.put(e, selected));
            SmartDashboard.putData(widget);
        }
        System.out.println("===============================================================");
    }

    /** Load the experiments class and thus the chooser. */
    public void show() {
        System.out.println("Showing dashboard experiment selectors.");
    }

    /** Override enablement, should be used for unit tests only. */
    public void override(Experiment experiment, boolean state) {
        m_enabled.put(experiment, state);
    }

    /**
     * Remember not to use this in the initial instantiation flow, since the
     * experiment twiddlers will have no effect later!
     */
    public boolean enabled(Experiment experiment) {
        return m_enabled.get(experiment);
    }

    ////////////////////////////////////////

    private String on(Experiment e) {
        return e.name() + " ON";
    }

    private String off(Experiment e) {
        return e.name() + " OFF";
    }
}
