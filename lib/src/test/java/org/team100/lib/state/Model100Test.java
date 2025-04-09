package org.team100.lib.state;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class Model100Test {
    private static final double kDelta = 0.001;

    @Test
    void testInterpolation1() {
        Model100 s0 = new Model100();
        Model100 s1 = new Model100(1, 0);
        Model100 lerp = s0.interpolate(s1, 0.5);
        assertEquals(0.5, lerp.x(), kDelta);
        assertEquals(0, lerp.v(), kDelta);
    }

    @Test
    void testInterpolation2() {
        // note this is *just* linear interpolation, it doesn't try to do the right
        // thing in phase space.
        Model100 s0 = new Model100();
        Model100 s1 = new Model100(1, 1);
        Model100 lerp = s0.interpolate(s1, 0.5);
        assertEquals(0.5, lerp.x(), kDelta);
        assertEquals(0.5, lerp.v(), kDelta);
    }
}
