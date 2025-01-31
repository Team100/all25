package org.team100.lib.commands.drivetrain.for_testing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.RealisticFixtured;
import org.team100.lib.testing.Timeless;

class OscillateFieldRelativeTest extends RealisticFixtured implements Timeless {
    private static final double kDelta = 0.001;

    @Test
    void testPosition() {
        OscillateFieldRelative od = new OscillateFieldRelative(fixture.logger, fixture.drive);
        od.initialize();
        assertEquals(0, od.m_initial.x().x(), kDelta);
        assertEquals(0, fixture.drive.getState().x().x(), kDelta);

        // half wave is 2 seconds
        for (int i = 0; i < 101; ++i) {
            stepTime(0.02);
            od.execute();
            fixture.drive.periodic();
        }

        // stopped
        assertEquals(0, fixture.drive.getState().x().v(), kDelta);
        // travel exactly 1 m.
        assertEquals(1, fixture.drive.getState().x().x(), kDelta);
    }
}
