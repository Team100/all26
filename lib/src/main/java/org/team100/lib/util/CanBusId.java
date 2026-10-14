package org.team100.lib.util;

/**
 * A wrapper for an int, indicating which of the systemcore CAN bus terminals to
 * use.
 * 
 * Use "find references" or "search" to find instances, e.g. "CanBusId(2)".
 */
public class CanBusId {
    public final int id;

    public CanBusId(int id) {
        this.id = id;
    }

    @Override
    public String toString() {
        return "CanBusId [id=" + id + "]";
    }

}
