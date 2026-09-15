package org.team100.lib.gtsam;

import org.junit.jupiter.api.Test;

public class SimTest {
    @Test
    void testSim() {
        System.out.println("========= construct sim ==========");
        Sim sim = new Sim();
        System.out.println("========= run sim ==========");
        sim.run();
        System.out.println("========= done! ==========");
    }
}
