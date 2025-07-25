package org.team100.lib.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class HomingRotaryPositionSensorTest {
    private static final double kDelta = 0.001;

    @Test
    void testHome() {
        MockRotaryPositionSensor s = new MockRotaryPositionSensor();
        HomingRotaryPositionSensor h = new HomingRotaryPositionSensor(s);
        assertEquals(0, h.getPositionRad().getAsDouble(), kDelta);
        h.setPosition(1);
        assertEquals(1, h.getPositionRad().getAsDouble(), kDelta);
        s.angle = 1;
        assertEquals(2, h.getPositionRad().getAsDouble(), kDelta);
    }
}
