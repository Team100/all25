package org.team100.lib.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.team100.lib.async.Async;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Creates a chooser based on the enum, slowly polls it, notifying the consumer
 * of updates.
 */
public class PolledEnumChooser<T extends Enum<T>> {
    private static final boolean DEBUG = false;
    private final String m_name;
    private final SendableChooser<T> m_chooser;
    private final List<Consumer<T>> m_consumers;
    private T m_selected;

    public PolledEnumChooser(
            Async async,
            Class<T> type,
            String name,
            T defaultValue) {
        m_name = name;
        m_chooser = new NamedChooser<T>(name);
        m_consumers = new ArrayList<>();
        for (T t : type.getEnumConstants()) {
            m_chooser.addOption(t.name(), t);
        }
        m_chooser.setDefaultOption(defaultValue.name(), defaultValue);
        SmartDashboard.putData(m_chooser);
        update();
        async.addPeriodic(this::update, 1, "PolledEnumChooser");
    }

    public void register(Consumer<T> consumer) {
        m_consumers.add(consumer);
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
            if (DEBUG)
                Util.printf("update %s (%d consumers) %s\n", m_name, m_consumers.size(), selected);
            for (Consumer<T> consumer : m_consumers) {
                if (DEBUG)
                    Util.printf("consumer %s\n", consumer);
                consumer.accept(selected);
            }
            m_selected = selected;
        }
    }

}
