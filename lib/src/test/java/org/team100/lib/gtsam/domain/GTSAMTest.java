package org.team100.lib.gtsam.domain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import gtsam.Point2;

public class GTSAMTest {

    @Test
    void testGtsam() throws Throwable {
        Point2 p = new Point2(4, 5);
        assertEquals(4, p.x());
        assertEquals(5, p.y());
    }
}
