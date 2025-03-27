package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class SetElevatorBarge extends Command {
    private final Elevator m_elevator;

    public SetElevatorBarge(Elevator elevator) {
        m_elevator = elevator;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevator.setPosition(0);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
