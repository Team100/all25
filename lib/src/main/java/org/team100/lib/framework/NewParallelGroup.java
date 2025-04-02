package org.team100.lib.framework;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A copy of WPILib's ParallelCommandGroup that logs what it's doing.
 */
public class NewParallelGroup extends Command {
    private final Map<Command, Boolean> m_commands = new LinkedHashMap<>();
    private final Map<Command, BooleanLogger> m_log_active = new LinkedHashMap<>();
    private final LoggerFactory m_logger;
    private final String m_name;

    private boolean m_runWhenDisabled = true;
    private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

    public NewParallelGroup(LoggerFactory logger, String name, Command... commands) {
        m_logger = logger.child(name);
        m_name = name;
        addCommands(commands);
    }

    public final void addCommands(Command... commands) {
        if (m_commands.containsValue(true)) {
            throw new IllegalStateException(
                    "Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            if (!Collections.disjoint(command.getRequirements(), getRequirements())) {
                throw new IllegalArgumentException(
                        "Multiple commands in a parallel composition cannot require the same subsystems");
            }
            m_log_active.put(command, m_logger.booleanLogger(Level.TRACE, command.getName()));
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
            Command cmd = commandRunning.getKey();
            cmd.initialize();
            commandRunning.setValue(true);
            m_log_active.get(cmd).log(() -> true);
        }
    }

    @Override
    public final void execute() {
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            if (!commandRunning.getValue()) {
                continue;
            }
            Command cmd = commandRunning.getKey();
            cmd.execute();
            if (cmd.isFinished()) {
                cmd.end(false);
                commandRunning.setValue(false);
                m_log_active.get(cmd).log(() -> false);
            }
        }
    }

    @Override
    public final void end(boolean interrupted) {
        if (interrupted) {
            for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
                if (commandRunning.getValue()) {
                    Command cmd = commandRunning.getKey();
                    m_log_active.get(cmd).log(() -> false);
                    cmd.end(true);
                }
            }
        }
    }

    @Override
    public final boolean isFinished() {
        return !m_commands.containsValue(true);
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
    public String getName() {
        return m_name;
    }
}
