package org.team100.lib.indicator;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.driverstation.Alert;



/**
 * A container for alerts put up on the dashboard, and a condition that can be
 * used to draw attention to the alerts (e.g. by setting the LED state)
 */
public class Alerts {
    private final List<Alert> m_alerts;

    public Alerts() {
        m_alerts = new ArrayList<>();
    }

    public Alert add(String text, Alert.Level type) {
        Alert alert = new Alert(text, type);
        m_alerts.add(alert);
        return alert;
    }

    public boolean any() {
        return m_alerts.stream().anyMatch(x -> x.get());
    }

}
