package org.team100.lib.util;

import java.util.function.Consumer;

import org.team100.lib.async.Async;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Creates a chooser based on the enum, slowly polls it, notifying the consumer
 * of updates.
 */
public class PolledEnumChooser<T extends Enum<T>> {
    private final SendableChooser<T> m_chooser;
    private final Consumer<T> m_consumer;
    private T m_selected;

    public PolledEnumChooser(
            Async async,
            Class<T> type,
            String name,
            T defaultValue,
            Consumer<T> consumer) {
        m_chooser = new NamedChooser<T>(name);
        m_consumer = consumer;
        for (T t : type.getEnumConstants()) {
            m_chooser.addOption(t.name(), t);
        }
        m_chooser.setDefaultOption(defaultValue.name(), defaultValue);
        SmartDashboard.putData(m_chooser);
        update();
        async.addPeriodic(this::update, 1, "PolledEnumChooser");
    }

    public T get() {
        return m_selected;
    }

    public void close() {
        m_chooser.close();
    }

    private void update() {
        T selected = m_chooser.getSelected();
        if (selected == null)
            return;
        if (selected != m_selected) {
            m_consumer.accept(selected);
            m_selected = selected;
        }
    }

}
