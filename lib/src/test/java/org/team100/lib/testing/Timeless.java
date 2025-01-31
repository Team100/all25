package org.team100.lib.testing;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.team100.lib.util.Memo;
import org.team100.lib.util.Takt;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/**
 * Sets up FPGA time stepping for unit tests.
 * 
 * Pausing the timer makes the tests deterministic.
 */
public interface Timeless {

    @BeforeEach
    default void pauseTiming() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        Takt.update();
    }

    @AfterEach
    default void resumeTiming() {
        SimHooks.resumeTiming();
        HAL.shutdown();
    }

    default void stepTime(double t) {
        SimHooks.stepTiming(t);
        Takt.update();
        Memo.resetAll();
    }

}
