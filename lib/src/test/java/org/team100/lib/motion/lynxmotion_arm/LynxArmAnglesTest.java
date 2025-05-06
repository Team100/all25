package org.team100.lib.motion.lynxmotion_arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class LynxArmAnglesTest {

    private static final double kDelta = 0.001;

    @Test
    public void testSwingRad() {
        assertEquals(Math.PI / 2, new LynxArmAngles.Factory().from0_1(0, 0, 0, 0, 0, 0).swingRad(), kDelta);
        assertEquals(Math.PI / 4, new LynxArmAngles.Factory().from0_1(0.25, 0, 0, 0, 0, 0).swingRad(), kDelta);
        assertEquals(0, new LynxArmAngles.Factory().from0_1(0.5, 0, 0, 0, 0, 0).swingRad(), kDelta);
        assertEquals(-Math.PI / 4, new LynxArmAngles.Factory().from0_1(0.75, 0, 0, 0, 0, 0).swingRad(), kDelta);
        assertEquals(-Math.PI / 2, new LynxArmAngles.Factory().from0_1(1, 0, 0, 0, 0, 0).swingRad(), kDelta);

        assertEquals(0, new LynxArmAngles.Factory().fromRad(Math.PI / 2, 0, 0, 0, 0, 0).swing, kDelta);
        assertEquals(0.25, new LynxArmAngles.Factory().fromRad(Math.PI / 4, 0, 0, 0, 0, 0).swing, kDelta);
        assertEquals(0.5, new LynxArmAngles.Factory().fromRad(0, 0, 0, 0, 0, 0).swing, kDelta);
        assertEquals(0.75, new LynxArmAngles.Factory().fromRad(-Math.PI / 4, 0, 0, 0, 0, 0).swing, kDelta);
        assertEquals(1, new LynxArmAngles.Factory().fromRad(-Math.PI / 2, 0, 0, 0, 0, 0).swing, kDelta);
    }

    @Test
    public void testBoomRad() {
        assertEquals(1, new LynxArmAngles.Factory().from0_1(0, 0.25, 0, 0, 0, 0).boomRad(), kDelta);
        assertEquals(0, new LynxArmAngles.Factory().from0_1(0, 0.5, 0, 0, 0, 0).boomRad(), kDelta);
        assertEquals(-1, new LynxArmAngles.Factory().from0_1(0, 0.75, 0, 0, 0, 0).boomRad(), kDelta);

        assertEquals(0.75, new LynxArmAngles.Factory().fromRad(0, -1, 0, 0, 0, 0).boom, kDelta);
        assertEquals(0.5, new LynxArmAngles.Factory().fromRad(0, 0, 0, 0, 0, 0).boom, kDelta);
        assertEquals(0.25, new LynxArmAngles.Factory().fromRad(0, 1, 0, 0, 0, 0).boom, kDelta);
    }

    @Test
    public void testStickRad() {
        assertEquals(Math.PI / 4, new LynxArmAngles.Factory().from0_1(0, 0, 0.25, 0, 0, 0).stickRad(), kDelta);
        assertEquals(Math.PI / 2, new LynxArmAngles.Factory().from0_1(0, 0, 0.5, 0, 0, 0).stickRad(), kDelta);
        assertEquals(Math.PI, new LynxArmAngles.Factory().from0_1(0, 0, 1, 0, 0, 0).stickRad(), kDelta);

        assertEquals(0.25, new LynxArmAngles.Factory().fromRad(0, 0, Math.PI / 4, 0, 0, 0).stick, kDelta);
        assertEquals(0.5, new LynxArmAngles.Factory().fromRad(0, 0, Math.PI / 2, 0, 0, 0).stick, kDelta);
        assertEquals(1, new LynxArmAngles.Factory().fromRad(0, 0, Math.PI, 0, 0, 0).stick, kDelta);
    }

    @Test
    public void testWristRad() {
        LynxArmAngles.Config config = new LynxArmAngles.Config();
        LynxArmAngles.Factory factory = new LynxArmAngles.Factory(config);

        assertEquals(-Math.PI / 4, factory.from0_1(0, 0, 0, config.wristCenter + 0.25, 0, 0).wristRad(), kDelta);
        assertEquals(0, factory.from0_1(0, 0, 0, config.wristCenter, 0, 0).wristRad(), kDelta);
        assertEquals(Math.PI / 4, factory.from0_1(0, 0, 0, config.wristCenter - 0.25, 0, 0).wristRad(), kDelta);

        assertEquals(config.wristCenter + 0.25, factory.fromRad(0, 0, 0, -Math.PI / 4, 0, 0).wrist, kDelta);
        assertEquals(config.wristCenter, factory.fromRad(0, 0, 0, 0, 0, 0).wrist, kDelta);
        assertEquals(config.wristCenter - 0.25, factory.fromRad(0, 0, 0, Math.PI / 4, 0, 0).wrist, kDelta);

    }

}