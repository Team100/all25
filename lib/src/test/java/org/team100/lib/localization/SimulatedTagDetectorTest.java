package org.team100.lib.localization;

import org.junit.jupiter.api.Test;

public class SimulatedTagDetectorTest {
    @Test
    void testSimple() {
        SimulatedTagDetector sim = new SimulatedTagDetector();
        sim.periodic();
    }
}
