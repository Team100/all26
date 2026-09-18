package org.team100.lib.gtsam.field;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import gtsam.Point3;

/**
 * Field for simulation
 * see field_map.py in all24.
 */
public class FieldMap {
    static double TAG_SIZE_M = 0.1651;
    static double HALF = TAG_SIZE_M / 2;
    // key = tag number
    // value = list of arrays, each array is (x, y)
    // TODO: make this a flat array with 8 numbers instead.
    Map<Integer, List<Point3>> tags;

    public FieldMap() throws Throwable {
        tags = new TreeMap<>();
        // tag zero is like something we could set up for practice, 1m high
        tags.put(0, FieldMap.make_tag(8, 4, 1, 0));
        tags.put(1, FieldMap.make_tag(2, 2, 1, Math.PI / 2));
    }

    /**
     * list of corners
     *
     * the order is the same as the detector getCorners order
     *
     * lower left
     * lower right
     * upper right
     * upper left
     * 
     */
    public List<Point3> get(int tag_id) {
        return tags.get(tag_id);
    }

    /**
     * yaw: 'into the page' orientation.
     * pitch and roll are always zero
     * 
     * @throws Throwable
     */
    static List<Point3> make_tag(double x, double y, double z, double yaw) throws Throwable {
        double s = HALF * Math.sin(yaw);
        double c = HALF * Math.cos(yaw);
        Point3 ll = new Point3(x - s, y + c, z - HALF);
        Point3 lr = new Point3(x + s, y - c, z - HALF);
        Point3 ur = new Point3(x + s, y - c, z + HALF);
        Point3 ul = new Point3(x - s, y + c, z + HALF);
        return List.of(ll, lr, ur, ul);
    }

}
