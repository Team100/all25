package org.team100.lib.framework;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A copy of WPILib's SequentialCommandGroup that logs what it's doing.
 */
public class SequentialCommandGroup100 extends Command {
    private final List<Command> m_commands = new ArrayList<>();
    private final List<BooleanLogger> m_log_active = new ArrayList<>();
    protected final LoggerFactory m_logger;
    private final String m_name;

    private int m_currentCommandIndex = -1;
    private boolean m_runWhenDisabled = true;
    private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

    /**
     * It would be good to use a named child as the logger here, so that we don't
     * have so many publishers to the same log key.
     */
    public SequentialCommandGroup100(LoggerFactory logger, String name, Command... commands) {
        m_logger = logger.child(name);
        m_name = name;
        addCommands(commands);
    }

    public final void addCommands(Command... commands) {
        if (m_currentCommandIndex != -1) {
            throw new IllegalStateException(
                    "Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            m_log_active.add(m_logger.booleanLogger(Level.TRACE, command.getName()));
            m_commands.add(command);
            addRequirements(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
            if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                m_interruptBehavior = InterruptionBehavior.kCancelSelf;
            }
        }
    }

    @Override
    public final void initialize() {
        m_currentCommandIndex = 0;

        if (!m_commands.isEmpty()) {
            m_commands.get(0).initialize();
        }
    }

    @Override
    public final void execute() {
        if (m_commands.isEmpty()) {
            return;
        }
        Command currentCommand = m_commands.get(m_currentCommandIndex);
        m_log_active.get(m_currentCommandIndex).log(() -> true);
        currentCommand.execute();
        if (currentCommand.isFinished()) {
            m_log_active.get(m_currentCommandIndex).log(() -> false);
            currentCommand.end(false);
            m_currentCommandIndex++;
            if (m_currentCommandIndex < m_commands.size()) {
                m_commands.get(m_currentCommandIndex).initialize();
            }
        }
    }

    @Override
    public final void end(boolean interrupted) {
        if (interrupted
                && !m_commands.isEmpty()
                && m_currentCommandIndex > -1
                && m_currentCommandIndex < m_commands.size()) {
            m_log_active.get(m_currentCommandIndex).log(() -> false);
            m_commands.get(m_currentCommandIndex).end(true);
        }
        m_currentCommandIndex = -1;

        // System.out.println("I AM FINISHING MY COMMAND GROUP 100");
    }

    @Override
    public final boolean isFinished() {
        return m_currentCommandIndex == m_commands.size();
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
