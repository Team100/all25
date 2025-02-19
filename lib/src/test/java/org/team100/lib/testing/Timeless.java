package org.team100.lib.testing;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.util.Memo;
import org.team100.lib.util.Takt;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/**
 * Sets up FPGA time stepping for unit tests.
 * 
 * Pausing the timer makes the tests deterministic.
 * 
 * use stepTime() to simulate the passage of time.
 * 
 * You'll also need to reset whatever caches you use, perhaps in their periodic.
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

    /**
     * Increments the clock and resets all the memoized quantities.
     * This used to allow the time step to be specified, but it's not a realistic to
     * require correctness in that case, so I took it out.
     */
    default void stepTime() {
        SimHooks.stepTiming(TimedRobot100.LOOP_PERIOD_S);
        Takt.update();
        Memo.resetAll();
        Memo.updateAll();
    }

}
