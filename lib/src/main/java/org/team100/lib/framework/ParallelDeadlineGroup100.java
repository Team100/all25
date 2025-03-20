package org.team100.lib.framework;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.StringLogger;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A copy of WPILib's ParallelDeadlineGroup that logs what it's doing.
 */
public class ParallelDeadlineGroup100 extends Command {
    private final Map<Command, Boolean> m_commands = new LinkedHashMap<>();
    private boolean m_runWhenDisabled = true;
    private boolean m_finished = true;
    private Command m_deadline;
    private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;
    private final StringLogger m_log_active_commands;

    public ParallelDeadlineGroup100(LoggerFactory parent, Command deadline, Command... otherCommands) {
        m_log_active_commands = parent.stringLogger(Level.TRACE, "active commands");
        setDeadline(deadline);
        addCommands(otherCommands);
    }

    public final void setDeadline(Command deadline) {
        @SuppressWarnings("PMD.CompareObjectsWithEquals")
        boolean isAlreadyDeadline = deadline == m_deadline;
        if (isAlreadyDeadline) {
            return;
        }
        if (m_commands.containsKey(deadline)) {
            throw new IllegalArgumentException(
                    "The deadline command cannot also be in the other commands!");
        }
        addCommands(deadline);
        m_deadline = deadline;
    }

    public final void addCommands(Command... commands) {
        if (!m_finished) {
            throw new IllegalStateException(
                    "Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            if (!Collections.disjoint(command.getRequirements(), getRequirements())) {
                throw new IllegalArgumentException(
                        "Multiple commands in a parallel group cannot require the same subsystems");
            }
            m_commands.put(command, false);
            addRequirements(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
            if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                m_interruptBehavior = InterruptionBehavior.kCancelSelf;
            }
        }
    }

    @Override
    public final void initialize() {
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            commandRunning.getKey().initialize();
            commandRunning.setValue(true);
        }
        m_finished = false;
    }

    @Override
    public final void execute() {
        StringBuilder activeCommands = new StringBuilder();
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            if (!commandRunning.getValue()) {
                continue;
            }
            activeCommands.append(":");
            activeCommands.append(commandRunning.getKey().getName());
            commandRunning.getKey().execute();
            if (commandRunning.getKey().isFinished()) {
                commandRunning.getKey().end(false);
                commandRunning.setValue(false);
                if (commandRunning.getKey().equals(m_deadline)) {
                    m_finished = true;
                }
            }
        }
        m_log_active_commands.log(activeCommands::toString);
    }

    @Override
    public final void end(boolean interrupted) {
        m_log_active_commands.log(() -> "");
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            if (commandRunning.getValue()) {
                commandRunning.getKey().end(true);
            }
        }
    }

    @Override
    public final boolean isFinished() {
        return m_finished;
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_runWhenDisabled;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return m_interruptBehavior;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addStringProperty("deadline", m_deadline::getName, null);
    }
}
