package org.team100.frc2025;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class VCGTest {
    private static final double kDelta = 0.001;

    @Test
    void testDown() {
        double elevatorPosition = 0;
        assertEquals(0.18, VCG.vcg(elevatorPosition), kDelta);
    }

    @Test
    void testUp() {
        double elevatorPosition = 2;
        assertEquals(0.58, VCG.vcg(elevatorPosition), kDelta);
    }

}
