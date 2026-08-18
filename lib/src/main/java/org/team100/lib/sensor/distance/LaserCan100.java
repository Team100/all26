package org.team100.lib.sensor.distance;

import org.team100.lib.util.CanId;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.simulation.MockLaserCan;

/**
 * Wrapper around the LaserCan class, so we can use CanId.
 */
public class LaserCan100 {
    private final LaserCanInterface m_sensor;

    /** For testing only. */
    public LaserCan100() {
        this(new MockLaserCan());
    }

    public LaserCan100(CanId id) {
        this(new LaserCan(id.id));
    }

    private LaserCan100(LaserCanInterface lc) {
        m_sensor = lc;
    }

    public Measurement getMeasurement() {
        return m_sensor.getMeasurement();
    }

}
