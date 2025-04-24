package org.team100.lib.framework;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Illustrates an issue with triggers: the "initialize" method of scheduled
 * commands is run immediately upon scheduling, and the triggers schedule
 * commands immediately upon evaluating their conditions. This means that
 * triggers that modify conditions (as are common in state machines) can have
 * surprising effects: "later" trigger evaluations can miss conditions because
 * the "earlier" evaluations have changed the conditions. The evaluation order
 * is insertion order.
 * 
 * see https://www.chiefdelphi.com/t/trigger-condition-evaluation/500280
 * 
 */
public class TriggerTest {
    boolean foo;
    boolean A;
    boolean B;

    /**
     * This illustrates a workaround suggested by Sam, using "run.until" instead of
     * "runOnce".
     * 
     * I don't like this solution, because it forces you to remember which one to
     * use, and also because it doesn't work for "real" commands with "initialize"
     * logic.
     * 
     * see
     * https://github.com/wpilibsuite/allwpilib/issues/7920#issuecomment-2827375076
     */
    @Test
    void testThatWorks() {
        HAL.initialize(500, 0);
        CommandScheduler scheduler = CommandScheduler.getInstance();
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        A = false;
        B = false;
        new Trigger(() -> foo)
                .onTrue(run(
                        // this runs in execute, which happens after all the triggers have been
                        // evaluated, so the falling edge of foo is visible to the binding below.
                        () -> {
                            foo = false;
                            A = true;
                        }).until(() -> true))
                .onFalse(runOnce(() -> {
                    B = true;
                }));
        foo = true;
        assertFalse(A);
        scheduler.run();
        assertTrue(A);
        assertFalse(B);
        scheduler.run();
        assertTrue(B);
    }

    @Test
    void testThatDoesNotWork() {
        HAL.initialize(500, 0);
        CommandScheduler scheduler = CommandScheduler.getInstance();
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        A = false;
        B = false;
        new Trigger(() -> foo)
                .onTrue(runOnce(
                        // this runs in initialize, which happens during trigger evaluation, so the
                        // falling edge of foo is not seen by the binding below.
                        () -> {
                            foo = false;
                            A = true;
                        }))
                .onFalse(runOnce(() -> {
                    // this never runs
                    B = true;
                }));

        foo = true;
        assertFalse(A);
        scheduler.run();
        assertTrue(A);
        assertFalse(B);
        scheduler.run();
        // this is the broken case.
        assertFalse(B);
    }
}
