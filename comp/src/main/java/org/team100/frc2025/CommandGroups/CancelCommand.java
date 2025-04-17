package org.team100.frc2025.CommandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class CancelCommand extends Command {
    Command m_command;
    boolean finished = false;

    public CancelCommand(Command command) {
        m_command = command;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancel(m_command);
        finished = true;
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        finished = false;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
