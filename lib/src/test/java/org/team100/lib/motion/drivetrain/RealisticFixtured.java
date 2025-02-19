package org.team100.lib.motion.drivetrain;

import org.junit.jupiter.api.AfterEach;

/**
 * Uses simulated position sensors, must be used with clock control (e.g.
 * {@link Timeless}).
 */
public class RealisticFixtured {
    protected RealisticFixture fixture = new RealisticFixture();

    @AfterEach
    void close() {
        fixture.close();
    }
}
