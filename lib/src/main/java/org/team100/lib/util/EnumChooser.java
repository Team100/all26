package org.team100.lib.util;

import java.util.EnumSet;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnumChooser<T extends Enum<T>> {

    private final SendableChooser<T> m_chooser;

    public EnumChooser(String name, T defaultOption) {
        m_chooser = new NamedChooser<>(name);
        for (T level : EnumSet.allOf(defaultOption.getDeclaringClass())) {
            m_chooser.addOption(level.name(), level);
        }
        m_chooser.setDefaultOption(defaultOption.name(), defaultOption);
        SmartDashboard.putData(m_chooser);
    }

    public T get() {
        return m_chooser.getSelected();
    }

}
