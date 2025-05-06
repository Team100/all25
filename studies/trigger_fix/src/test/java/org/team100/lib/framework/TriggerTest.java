package org.team100.lib.framework;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.event.EventLoop;
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
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        A = false;
        B = false;
        EventLoop loop = new EventLoop();
        new Trigger(loop, () -> foo)
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
        CommandScheduler scheduler = CommandScheduler.getInstance();
        foo = true;
        assertFalse(A);
        loop.poll();
        scheduler.run();
        assertTrue(A);
        assertFalse(B);
        loop.poll();
        scheduler.run();
        assertTrue(B);
    }

    @Test
    void testThatDoesNotWork() {
        HAL.initialize(500, 0);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        A = false;
        B = false;
        EventLoop loop = new EventLoop();
        new Trigger(loop, () -> foo)
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

        CommandScheduler scheduler = CommandScheduler.getInstance();
        foo = true;
        assertFalse(A);
        loop.poll();
        scheduler.run();
        assertTrue(A);
        assertFalse(B);
        loop.poll();
        scheduler.run();
        // this is the broken case.
        assertFalse(B);
    }

    /*
     * Here's an alternative way to fix it, that doesn't require changing the
     * trigger. Instead, guard the state. Accept updates anytime, and commit
     * explicitly (this would go in robotperiodic or whatever).
     */
    static class State {
        private boolean value;
        private boolean newValue;

        public void set(boolean x) {
            newValue = x;
        }

        public boolean get() {
            return value;
        }

        public void commit() {
            value = newValue;
        }
    }

    State FOO = new State();

    @Test
    void testAlternative() {
        HAL.initialize(500, 0);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        A = false;
        B = false;
        EventLoop loop = new EventLoop();
        new Trigger(loop, FOO::get)
                .onTrue(runOnce(
                        // this still runs immediately, but it just sets the newValue.
                        () -> {
                            FOO.set(false);
                            A = true;
                        }))
                .onFalse(runOnce(() -> {
                    // this now runs
                    B = true;
                }));

        CommandScheduler scheduler = CommandScheduler.getInstance();
        FOO.set(true);
        FOO.commit();
        assertFalse(A);
        loop.poll();
        scheduler.run();
        FOO.commit();
        assertTrue(A);
        assertFalse(B);
        loop.poll();
        scheduler.run();
        FOO.commit();
        // and now it's fixed
        assertTrue(B);
    }

}
