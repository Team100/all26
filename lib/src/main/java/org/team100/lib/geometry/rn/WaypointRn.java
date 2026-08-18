package org.team100.lib.geometry.rn;

import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

/**
 * Note: velocity is used for the spline endpoint, it's not exactly
 * velocity in any formal sense.
 */
public record WaypointRn<N extends Num>(
        Vector<N> position, Vector<N> velocity) {

    public WaypointRn(Vector<N> position, Vector<N> velocity) {
        this.position = position;
        this.velocity = velocity;
    }

    public int dim() {
        return position.getNumRows();
    }

    @Override
    public String toString() {
        return String.format("WaypointRn [%s %s]",
                StrUtil.vecStr(position), StrUtil.vecStr(velocity));
    }

}
