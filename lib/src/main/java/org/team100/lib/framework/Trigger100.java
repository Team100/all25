package org.team100.lib.framework;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Similar to WPILib Trigger, without the bug
 * https://github.com/wpilibsuite/allwpilib/issues/7920
 * 
 * I took out the static default button loop constructor.
 * Supply your dependencies, yo.
 * 
 * I took out the null checker.
 * Don't supply nulls, yo.
 */
public class Trigger100 implements BooleanSupplier {
    @FunctionalInterface
    private interface BindingBody {
        void run(boolean previous, boolean current);
    }

    private final BooleanSupplier m_condition;
    private final EventLoop100 m_loop;

    public Trigger100(EventLoop100 loop, BooleanSupplier condition) {
        m_loop = loop;
        m_condition = condition;
    }

    public Trigger100 onChange(Command command) {
        addBinding(
                (previous, current) -> {
                    if (previous != current) {
                        command.schedule();
                    }
                });
        return this;
    }

    public Trigger100 onTrue(Command command) {
        addBinding(
                (previous, current) -> {
                    if (!previous && current) {
                        command.schedule();
                    }
                });
        return this;
    }

    public Trigger100 onFalse(Command command) {
        addBinding(
                (previous, current) -> {
                    if (previous && !current) {
                        command.schedule();
                    }
                });
        return this;
    }

    public Trigger100 whileTrue(Command command) {
        addBinding(
                (previous, current) -> {
                    if (!previous && current) {
                        command.schedule();
                    }
                });
        addBinding(
                (previous, current) -> {
                    if (previous && !current) {
                        command.cancel();
                    }
                });

        return this;
    }

    public Trigger100 whileFalse(Command command) {
        addBinding(
                (previous, current) -> {
                    if (previous && !current) {
                        command.schedule();
                    }
                });
        addBinding(
                (previous, current) -> {
                    if (!previous && current) {
                        command.cancel();
                    }
                });
        return this;
    }

    public Trigger100 toggleOnTrue(Command command) {
        addBinding(
                (previous, current) -> {
                    if (!previous && current) {
                        if (command.isScheduled()) {
                            command.cancel();
                        } else {
                            command.schedule();
                        }
                    }
                });
        return this;
    }

    public Trigger100 toggleOnFalse(Command command) {
        addBinding(
                (previous, current) -> {
                    if (previous && !current) {
                        if (command.isScheduled()) {
                            command.cancel();
                        } else {
                            command.schedule();
                        }
                    }
                });
        return this;
    }

    public Trigger100 and(BooleanSupplier trigger) {
        return new Trigger100(m_loop, () -> m_condition.getAsBoolean() && trigger.getAsBoolean());
    }

    public Trigger100 or(BooleanSupplier trigger) {
        return new Trigger100(m_loop, () -> m_condition.getAsBoolean() || trigger.getAsBoolean());
    }

    public Trigger100 negate() {
        return new Trigger100(m_loop, () -> !m_condition.getAsBoolean());
    }

    public Trigger100 debounce(double seconds) {
        return debounce(seconds, Debouncer.DebounceType.kRising);
    }

    public Trigger100 debounce(double seconds, Debouncer.DebounceType type) {
        return new Trigger100(
                m_loop,
                new BooleanSupplier() {
                    final Debouncer m_debouncer = new Debouncer(seconds, type);

                    @Override
                    public boolean getAsBoolean() {
                        return m_debouncer.calculate(m_condition.getAsBoolean());
                    }
                });
    }

    @Override
    public boolean getAsBoolean() {
        return m_condition.getAsBoolean();
    }

    private void addBinding(BindingBody body) {
        m_loop.bind(
                new Runnable() {
                    private boolean m_previous = m_condition.getAsBoolean();

                    @Override
                    public void run() {
                        boolean current = m_condition.getAsBoolean();

                        body.run(m_previous, current);

                        m_previous = current;
                    }
                });
    }
}
