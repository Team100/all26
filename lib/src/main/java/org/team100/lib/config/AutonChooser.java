package org.team100.lib.config;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;

import org.team100.lib.util.NamedChooser;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonChooser {

    private final NamedChooser<AnnotatedCommand> m_chooser;
    /** duplicate names are not allowed. */
    private final Set<String> m_names;

    public AutonChooser() {
        m_chooser = new NamedChooser<>("Auton Command");
        m_names = new HashSet<>();
        addAsDefault(new AnnotatedCommandImpl("NONE", null, null, null));
        SmartDashboard.putData(m_chooser);
    }

    public void addAsDefault(AnnotatedCommand cmd) {
        m_chooser.setDefaultOption(unique(cmd.name()), cmd);
    }

    public void add(AnnotatedCommand cmd) {
        m_chooser.addOption(unique(cmd.name()), cmd);
    }

    public AnnotatedCommand get() {
        return m_chooser.getSelected();
    }

    public void onChange(Consumer<AnnotatedCommand> listener) {
        m_chooser.onChange(listener);
    }

    public void close() {
        m_chooser.close();
    }

    private String unique(String name) {
        if (!m_names.add(name))
            throw new IllegalArgumentException("Duplicate name " + name);
        return name;
    }
}
