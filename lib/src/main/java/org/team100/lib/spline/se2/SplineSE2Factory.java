package org.team100.lib.spline.se2;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.se2_universal_cover.WindingWaypointSE2;

public class SplineSE2Factory {
    /**
     * Make N-1 splines from N waypoint knots.
     */
    public static List<SplineSE2> splinesFromWaypoints(List<WindingWaypointSE2> waypoints) {
        List<SplineSE2> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i0 = 0; i0 < waypoints.size() - 1; ++i0) {
            int i1 = i0 + 1;
            splines.add(new SplineSE2(waypoints.get(i0), waypoints.get(i1)));
        }
        return splines;
    }

}
