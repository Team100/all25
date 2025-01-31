package org.team100.frc2025;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.team100.lib.util.Memo;
import org.team100.lib.util.Takt;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/** A copy of Timeless from lib test, because frc2024 can't see it */
public interface Timeless2024 {

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
