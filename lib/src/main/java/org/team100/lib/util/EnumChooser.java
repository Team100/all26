package org.team100.lib.util;

import java.util.EnumSet;

public class EnumChooser<T extends Enum<T>> {

    private final NamedChooser<T> m_chooser;

    public EnumChooser(String name, T defaultOption) {
        m_chooser = new NamedChooser<>(name);
        for (T level : EnumSet.allOf(defaultOption.getDeclaringClass())) {
            m_chooser.add(level.name(), level);
        }
        m_chooser.addDefault(defaultOption.name(), defaultOption);
    }

    public T get() {
        return m_chooser.getSelected();
    }

}
