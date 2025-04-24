package org.team100.lib.framework;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashMap;
import java.util.Map;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Like TriggerTest but for Trigger100. */
public class Trigger100Test {
    boolean foo;
    boolean A;
    boolean B;

    @Test
    void testMap() {
        Map<String, Boolean> m = new HashMap<>();
        m.put("hi", false);
        assertFalse(m.get("hi"));
        for (Map.Entry<String, Boolean> entry : m.entrySet()) {
            entry.setValue(true);
        }
        assertTrue(m.get("hi"));
    }

    @Test
    void testThatWorks() {
        HAL.initialize(500, 0);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        A = false;
        B = false;
        EventLoop100 loop = new EventLoop100();
        new Trigger100(loop, () -> foo)
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
        EventLoop100 loop = new EventLoop100();
        new Trigger100(loop, () -> foo)
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
}
