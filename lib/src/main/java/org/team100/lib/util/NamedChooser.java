package org.team100.lib.util;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;

import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

/**
 * Makes it easier to set the name of a SendableChooser.
 */
public class NamedChooser<T> {
    private static final Set<String> names = new HashSet<>();
    private final Selectable<T> m_selectable;
    private final String m_name;

    /**
     * @param name must be globally unique.
     */
    public NamedChooser(String name) {
        if (names.contains(name))
            throw new IllegalArgumentException("duplicate named chooser: " + name);
        names.add(name);
        m_name = name;
        m_selectable = new Selectable<>();
        Tunables.remove(name);
        Tunables.publish(name, m_selectable);
    }

    public void add(String name, T object) {
        m_selectable.add(name, object);
    }

    public void addDefault(String name, T object) {
        m_selectable.addDefault(name, object);
    }

    public void onChange(Consumer<T> listener) {
        m_selectable.onChange(listener);
    }

    public T getSelected() {
        return m_selectable.getSelected();
    }

    public void close() {
        names.remove(m_name);
        Tunables.remove(m_name);
    }
}
