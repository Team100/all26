package org.team100.lib.experiments;

import java.util.EnumMap;
import java.util.Map;
import java.util.Set;

import org.team100.lib.config.Identity;

import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;

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
    public static final Experiments INSTANCE = new Experiments(Identity.instance);

    /** These experiments are enabled by default. */
    private final Set<Experiment> m_defaults = Set.of(
            Experiment.HeedVision);

    /** Key = experiment, value = enabled. */
    private final Map<Experiment, Boolean> m_enabled;

    private Experiments(Identity identity) {
        m_enabled = new EnumMap<>(Experiment.class);
        for (Experiment e : Experiment.values()) {
            SendableChooser<Boolean> widget = ExperimentChooser.get(e.name());
            if (m_defaults.contains(e)) {
                widget.setDefaultOption(on(e), true);
                widget.addOption(off(e), false);
                m_enabled.put(e, true);
            } else {
                widget.addOption(on(e), true);
                widget.setDefaultOption(off(e), false);
                m_enabled.put(e, false);
            }
            widget.onChange(selected -> m_enabled.put(e, selected));
            SmartDashboard.putData(widget);
        }
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
