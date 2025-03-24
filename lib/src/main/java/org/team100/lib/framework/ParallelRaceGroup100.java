package org.team100.lib.framework;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A copy of WPILib's ParallelGraceGroup that logs what it's doing.
 */
public class ParallelRaceGroup100 extends Command {
    private final Set<Command> m_commands = new LinkedHashSet<>();
    private final Set<BooleanLogger> m_log_active = new LinkedHashSet<>();
    private final LoggerFactory m_logger;
    private final String m_name;

    private boolean m_runWhenDisabled = true;
    private boolean m_finished = true;
    private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

    public ParallelRaceGroup100(LoggerFactory logger, String name, Command... commands) {
        m_logger = logger.child(name);
        m_name = name;
        addCommands(commands);
    }

    public final void addCommands(Command... commands) {
        if (!m_finished) {
            throw new IllegalStateException(
                    "Commands cannot be added to a composition while it's running!");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            if (!Collections.disjoint(command.getRequirements(), getRequirements())) {
                throw new IllegalArgumentException(
                        "Multiple commands in a parallel composition cannot require the same subsystems");
            }
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
        m_finished = false;
        for (Command command : m_commands) {
            command.initialize();
        }
        for (BooleanLogger logger : m_log_active) {
            logger.log(() -> true);
        }
    }

    @Override
    public final void execute() {
        for (Command command : m_commands) {
            command.execute();
            if (command.isFinished()) {
                m_finished = true;
            }
        }
    }

    @Override
    public final void end(boolean interrupted) {
        for (Command command : m_commands) {
            command.end(!command.isFinished());
        }
        for (BooleanLogger logger : m_log_active) {
            logger.log(() -> false);
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
    public String getName() {
        return m_name;
    }
}
